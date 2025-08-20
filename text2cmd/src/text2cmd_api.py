#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Text2Cmd模块 - 简化版本接口
提供易于使用的API接口用于其他模块调用
"""

import os
import sys
import json
from typing import Dict, List, Optional

# 添加当前目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

try:
    from text2cmd_parser import Text2CmdParser, ParsedCommand, CommandType, Position
except ImportError as e:
    print(f"导入错误: {e}")
    print("请确保已安装必要的依赖包")
    sys.exit(1)


class Text2CmdAPI:
    """Text2Cmd API接口类"""
    
    def __init__(self, config_path: Optional[str] = None):
        """
        初始化API
        Args:
            config_path: 配置文件路径，如果为None则使用默认配置
        """
        if config_path is None:
            config_path = os.path.join(current_dir, '..', 'config', 'text2cmd_config.yaml')
        
        self.parser = Text2CmdParser(config_path)
        print(f"Text2Cmd API初始化完成")
    
    def parse_text(self, text: str) -> Dict:
        """
        解析文本指令
        Args:
            text: 中文指令文本
        Returns:
            Dict: 解析结果字典
        """
        try:
            result = self.parser.parse(text)
            return self._command_to_dict(result)
        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "command_type": "stop",
                "confidence": 0.0
            }
    
    def parse_batch(self, texts: List[str]) -> List[Dict]:
        """
        批量解析文本指令
        Args:
            texts: 文本指令列表
        Returns:
            List[Dict]: 解析结果列表
        """
        results = []
        for text in texts:
            result = self.parse_text(text)
            results.append(result)
        return results
    
    def _command_to_dict(self, command: ParsedCommand) -> Dict:
        """将ParsedCommand转换为字典"""
        try:
            result = {
                "success": True,
                "command_id": command.command_id,
                "command_type": command.command_type.value,
                "timestamp": command.timestamp,
                "confidence": command.confidence,
                "original_text": command.original_text,
                "extracted_objects": command.extracted_objects,
                "wait_time": command.wait_time,
                "relative_movement": command.relative_movement,
                "force_control": command.force_control
            }
            
            # 添加位置信息
            if command.target_position:
                result["target_position"] = {
                    "x": command.target_position.x,
                    "y": command.target_position.y,
                    "z": command.target_position.z
                }
            else:
                result["target_position"] = None
            
            # 添加姿态信息
            if command.target_orientation:
                result["target_orientation"] = {
                    "x": command.target_orientation.x,
                    "y": command.target_orientation.y,
                    "z": command.target_orientation.z,
                    "w": command.target_orientation.w
                }
            else:
                result["target_orientation"] = None
            
            # 添加坐标系信息
            result["coordinate_system"] = command.coordinate_system.value
            
            # 添加运动类型
            result["motion_type"] = command.motion_type.value
            
            # 添加运动参数
            result["motion_params"] = {
                "speed": command.motion_params.speed,
                "acceleration": command.motion_params.acceleration,
                "deceleration": command.motion_params.deceleration,
                "jerk": command.motion_params.jerk,
                "blend_radius": command.motion_params.blend_radius
            }
            
            # 添加夹爪参数
            result["gripper_params"] = {
                "action": command.gripper_params.action,
                "force": command.gripper_params.force,
                "position": command.gripper_params.position,
                "speed": command.gripper_params.speed
            }
            
            return result
            
        except Exception as e:
            return {
                "success": False,
                "error": f"结果转换失败: {e}",
                "command_type": "stop",
                "confidence": 0.0
            }
    
    def validate_command(self, command_dict: Dict) -> tuple:
        """
        验证命令有效性
        Args:
            command_dict: 命令字典
        Returns:
            tuple: (is_valid, message)
        """
        try:
            # 检查必要字段
            if not command_dict.get("success", False):
                return False, "命令解析失败"
            
            # 检查置信度
            confidence = command_dict.get("confidence", 0.0)
            if confidence < 0.3:
                return False, f"置信度过低: {confidence:.2f}"
            
            # 检查命令类型
            command_type = command_dict.get("command_type")
            if not command_type:
                return False, "缺少命令类型"
            
            # 检查位置（对于需要位置的命令）
            if command_type in ["move", "pick", "place"]:
                position = command_dict.get("target_position")
                if not position:
                    return False, f"命令{command_type}需要目标位置"
            
            return True, "命令有效"
            
        except Exception as e:
            return False, f"验证失败: {e}"
    
    def get_supported_commands(self) -> List[str]:
        """获取支持的命令类型列表"""
        return [cmd.value for cmd in CommandType]
    
    def get_predefined_positions(self) -> Dict[str, List[float]]:
        """获取预定义位置列表"""
        return self.parser.position_keywords
    
    def set_workspace_limits(self, limits: Dict[str, List[float]]):
        """设置工作空间限制"""
        self.parser.workspace_limits = limits
        print(f"工作空间限制已更新: {limits}")


# 全局API实例
_api_instance = None

def get_api(config_path: Optional[str] = None) -> Text2CmdAPI:
    """获取API实例（单例模式）"""
    global _api_instance
    if _api_instance is None:
        _api_instance = Text2CmdAPI(config_path)
    return _api_instance


def parse_command(text: str) -> Dict:
    """
    快速解析命令的便捷函数
    Args:
        text: 中文指令文本
    Returns:
        Dict: 解析结果
    """
    api = get_api()
    return api.parse_text(text)


def parse_commands(texts: List[str]) -> List[Dict]:
    """
    批量解析命令的便捷函数
    Args:
        texts: 中文指令文本列表
    Returns:
        List[Dict]: 解析结果列表
    """
    api = get_api()
    return api.parse_batch(texts)


# 示例和测试函数
def demo():
    """演示Text2Cmd功能"""
    print("=== Text2Cmd模块演示 ===\n")
    
    # 创建API实例
    api = get_api()
    
    # 测试指令
    test_commands = [
        "缓慢地移动到坐标5，1，10",
        "快速移动到坐标-10,20,15",
        "抓取红色的苹果",
        "将物体放置到放置位置", 
        "回到初始位置",
        "停止所有动作",
        "张开夹爪",
        "以0.8的速度闭合夹爪",
        "等待3秒钟",
        "旋转到90度"
    ]
    
    print("支持的命令类型:", api.get_supported_commands())
    print("\n预定义位置:", list(api.get_predefined_positions().keys()))
    print("\n" + "="*60)
    
    # 逐个测试
    for i, cmd in enumerate(test_commands, 1):
        print(f"\n测试 {i}: {cmd}")
        result = api.parse_text(cmd)
        
        if result["success"]:
            print(f"✓ 命令类型: {result['command_type']}")
            print(f"✓ 置信度: {result['confidence']:.2f}")
            
            if result["target_position"]:
                pos = result["target_position"]
                print(f"✓ 目标位置: ({pos['x']}, {pos['y']}, {pos['z']})")
            
            print(f"✓ 运动速度: {result['motion_params']['speed']}")
            
            if result["extracted_objects"]:
                print(f"✓ 提取物体: {result['extracted_objects']}")
            
            # 验证命令
            is_valid, msg = api.validate_command(result)
            print(f"✓ 验证结果: {msg}")
            
        else:
            print(f"✗ 解析失败: {result.get('error', '未知错误')}")
        
        print("-" * 50)
    
    print("\n演示完成！")


def test_batch():
    """测试批量解析"""
    print("\n=== 批量解析测试 ===")
    
    commands = [
        "移动到原点",
        "抓取杯子",
        "放置到桌上",
        "回家"
    ]
    
    results = parse_commands(commands)
    
    for cmd, result in zip(commands, results):
        print(f"{cmd} -> {result['command_type']} (置信度: {result['confidence']:.2f})")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Text2Cmd模块测试")
    parser.add_argument("--demo", action="store_true", help="运行演示")
    parser.add_argument("--batch", action="store_true", help="运行批量测试")
    parser.add_argument("--text", type=str, help="解析指定文本")
    
    args = parser.parse_args()
    
    if args.demo:
        demo()
    elif args.batch:
        test_batch()
    elif args.text:
        result = parse_command(args.text)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        # 默认运行演示
        demo()
