#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Text2Cmd模块 - 简化演示版本
无需外部依赖，可以独立运行演示
"""

import json
import re
import uuid
from datetime import datetime
from typing import Dict, List, Optional, Tuple


class SimpleCommandType:
    """简化的命令类型"""
    MOVE = "move"
    PICK = "pick"
    PLACE = "place"
    STOP = "stop"
    HOME = "home"
    WAIT = "wait"
    OPEN_GRIPPER = "open_gripper"
    CLOSE_GRIPPER = "close_gripper"


class SimpleParser:
    """简化的指令解析器"""
    
    def __init__(self):
        """初始化解析器"""
        # 预定义位置
        self.positions = {
            "原点": [0.0, 0.0, 0.0],
            "零点": [0.0, 0.0, 0.0],
            "初始位置": [0.0, 0.0, 30.0],
            "起始位置": [0.0, 0.0, 30.0],
            "家位置": [0.0, 0.0, 30.0],
            "安全位置": [0.0, 0.0, 35.0],
            "休息位置": [0.0, -20.0, 15.0],
            "待机位置": [0.0, -20.0, 15.0],
            "抓取位置": [20.0, 10.0, 5.0],
            "取货位置": [20.0, 10.0, 5.0],
            "放置位置": [-20.0, 10.0, 5.0],
            "卸货位置": [-20.0, 10.0, 5.0],
            "观察位置": [0.0, 0.0, 40.0],
            "检测位置": [0.0, 15.0, 25.0]
        }
        
        # 速度关键词
        self.speeds = {
            "极慢": 0.1, "非常慢": 0.15, "很慢": 0.2, "慢": 0.3, "缓慢": 0.3,
            "慢速": 0.3, "低速": 0.3, "正常": 0.5, "一般": 0.5, "普通": 0.5,
            "中速": 0.5, "标准": 0.5, "快": 0.7, "快速": 0.7, "高速": 0.7,
            "很快": 0.8, "非常快": 0.9, "极快": 0.95, "最快": 1.0, "全速": 1.0
        }
        
        # 物体关键词
        self.objects = [
            "苹果", "香蕉", "橙子", "杯子", "瓶子", "盒子", "球", "积木",
            "螺丝", "零件", "工具", "物体", "物件", "东西", "货物", "产品"
        ]
        
        print("简化Text2Cmd解析器初始化完成")
    
    def parse(self, text: str) -> Dict:
        """
        解析文本指令
        Args:
            text: 中文指令文本
        Returns:
            Dict: 解析结果
        """
        try:
            # 预处理
            text = self._preprocess(text)
            
            # 生成命令ID
            command_id = str(uuid.uuid4())[:8]
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # 解析各个组件
            command_type = self._parse_command_type(text)
            position = self._parse_position(text)
            speed = self._parse_speed(text)
            objects = self._parse_objects(text)
            confidence = self._calculate_confidence(text, command_type, position)
            
            # 构建结果
            result = {
                "success": True,
                "command_id": command_id,
                "command_type": command_type,
                "timestamp": timestamp,
                "original_text": text,
                "target_position": position,
                "motion_params": {
                    "speed": speed,
                    "acceleration": 0.3,
                    "motion_type": "linear"
                },
                "gripper_params": {
                    "action": self._parse_gripper_action(text),
                    "force": 0.5,
                    "speed": 0.5
                },
                "extracted_objects": objects,
                "confidence": confidence,
                "coordinate_system": "world",
                "relative_movement": False,
                "force_control": False,
                "wait_time": self._parse_wait_time(text)
            }
            
            print(f"✓ 解析成功: {command_type} (置信度: {confidence:.2f})")
            return result
            
        except Exception as e:
            print(f"✗ 解析失败: {e}")
            return {
                "success": False,
                "error": str(e),
                "command_type": SimpleCommandType.STOP,
                "confidence": 0.0
            }
    
    def _preprocess(self, text: str) -> str:
        """预处理文本"""
        text = text.strip()
        text = text.replace('，', ',').replace('。', '').replace('！', '')
        text = re.sub(r'\s+', ' ', text)
        return text
    
    def _parse_command_type(self, text: str) -> str:
        """解析命令类型"""
        # 停止命令
        if any(word in text for word in ["停止", "暂停", "停", "stop"]):
            return SimpleCommandType.STOP
        
        # 回家命令
        if any(word in text for word in ["回家", "回到家", "初始位置", "起始位置", "home"]):
            return SimpleCommandType.HOME
        
        # 抓取命令
        if any(word in text for word in ["抓取", "抓", "拿", "夹", "取", "pick"]):
            return SimpleCommandType.PICK
        
        # 放置命令
        if any(word in text for word in ["放置", "放下", "放", "松开", "释放", "place"]):
            return SimpleCommandType.PLACE
        
        # 夹爪命令
        if any(word in text for word in ["张开夹爪", "打开夹爪", "张开"]):
            return SimpleCommandType.OPEN_GRIPPER
        if any(word in text for word in ["闭合夹爪", "关闭夹爪", "闭合"]):
            return SimpleCommandType.CLOSE_GRIPPER
        
        # 等待命令
        if any(word in text for word in ["等待", "暂停", "wait"]):
            return SimpleCommandType.WAIT
        
        # 默认移动命令
        return SimpleCommandType.MOVE
    
    def _parse_position(self, text: str) -> Optional[Dict[str, float]]:
        """解析位置信息"""
        # 检查预定义位置
        for keyword, pos in self.positions.items():
            if keyword in text:
                return {"x": pos[0], "y": pos[1], "z": pos[2]}
        
        # 解析数字坐标
        patterns = [
            r'坐标\s*([0-9.\-]+)\s*[,，]\s*([0-9.\-]+)\s*[,，]\s*([0-9.\-]+)',
            r'位置\s*([0-9.\-]+)\s*[,，]\s*([0-9.\-]+)\s*[,，]\s*([0-9.\-]+)',
            r'([0-9.\-]+)\s*[,，]\s*([0-9.\-]+)\s*[,，]\s*([0-9.\-]+)',
        ]
        
        for pattern in patterns:
            match = re.search(pattern, text)
            if match:
                try:
                    x, y, z = map(float, match.groups())
                    # 应用工作空间限制
                    x = max(-50, min(50, x))
                    y = max(-50, min(50, y))
                    z = max(0, min(50, z))
                    return {"x": x, "y": y, "z": z}
                except ValueError:
                    continue
        
        return None
    
    def _parse_speed(self, text: str) -> float:
        """解析速度信息"""
        for keyword, speed in self.speeds.items():
            if keyword in text:
                return speed
        return 0.5  # 默认速度
    
    def _parse_objects(self, text: str) -> List[str]:
        """解析物体信息"""
        objects = []
        for obj in self.objects:
            if obj in text:
                objects.append(obj)
        
        # 提取颜色修饰的物体
        color_pattern = r'([红绿蓝黄黑白橙紫粉灰棕]色?的?)([^，。！？\s]+)'
        matches = re.findall(color_pattern, text)
        for color, obj in matches:
            objects.append(f"{color}{obj}")
        
        return list(set(objects))
    
    def _parse_gripper_action(self, text: str) -> Optional[bool]:
        """解析夹爪动作"""
        if any(word in text for word in ["抓取", "抓", "夹", "取", "闭合"]):
            return True
        if any(word in text for word in ["放下", "松开", "释放", "张开"]):
            return False
        return None
    
    def _parse_wait_time(self, text: str) -> float:
        """解析等待时间"""
        wait_pattern = r'等待\s*([0-9.]+)\s*秒'
        match = re.search(wait_pattern, text)
        if match:
            return float(match.group(1))
        return 0.0
    
    def _calculate_confidence(self, text: str, command_type: str, position: Optional[Dict]) -> float:
        """计算置信度"""
        confidence = 0.5  # 基础置信度
        
        # 命令类型明确加分
        if any(word in text for word in ["移动", "抓取", "放置", "停止", "回家"]):
            confidence += 0.2
        
        # 有具体位置加分
        if position:
            confidence += 0.2
        
        # 有速度描述加分
        if any(speed in text for speed in self.speeds.keys()):
            confidence += 0.1
        
        # 有物体描述加分
        if any(obj in text for obj in self.objects):
            confidence += 0.1
        
        return min(1.0, confidence)


def demo():
    """演示简化版Text2Cmd功能"""
    print("=== 简化版Text2Cmd演示 ===\n")
    
    parser = SimpleParser()
    
    test_commands = [
        "缓慢地移动到坐标5，1，10",
        "快速移动到坐标-10,20,15",
        "抓取红色的苹果",
        "将物体放置到放置位置",
        "回到初始位置",
        "停止所有动作",
        "张开夹爪",
        "闭合夹爪",
        "等待3秒钟",
        "移动到观察位置"
    ]
    
    print("支持的预定义位置:", list(parser.positions.keys()))
    print("支持的速度关键词:", list(parser.speeds.keys()))
    print("\n" + "="*60)
    
    for i, cmd in enumerate(test_commands, 1):
        print(f"\n测试 {i}: {cmd}")
        result = parser.parse(cmd)
        
        if result["success"]:
            print(f"  命令类型: {result['command_type']}")
            print(f"  置信度: {result['confidence']:.2f}")
            
            if result["target_position"]:
                pos = result["target_position"]
                print(f"  目标位置: ({pos['x']}, {pos['y']}, {pos['z']})")
            
            print(f"  运动速度: {result['motion_params']['speed']}")
            
            if result["extracted_objects"]:
                print(f"  提取物体: {result['extracted_objects']}")
            
            if result["wait_time"] > 0:
                print(f"  等待时间: {result['wait_time']}秒")
        else:
            print(f"  解析失败: {result.get('error', '未知错误')}")
        
        print("-" * 40)
    
    print("\n演示完成！")


def interactive_test():
    """交互式测试"""
    print("=== 交互式测试模式 ===")
    print("输入中文指令进行测试，输入'quit'退出\n")
    
    parser = SimpleParser()
    
    while True:
        try:
            text = input("请输入指令: ").strip()
            if text.lower() in ['quit', 'exit', '退出']:
                break
            
            if not text:
                continue
            
            result = parser.parse(text)
            print(f"解析结果: {json.dumps(result, ensure_ascii=False, indent=2)}")
            print()
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"错误: {e}")
    
    print("测试结束")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "interactive":
            interactive_test()
        elif sys.argv[1] == "demo":
            demo()
        else:
            # 解析单个指令
            text = " ".join(sys.argv[1:])
            parser = SimpleParser()
            result = parser.parse(text)
            print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        demo()
