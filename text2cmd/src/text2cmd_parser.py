#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
高级AI指令解析器 - Text2Cmd模块
专门用于将中文自然语言指令转换为结构化的机械臂控制命令

支持的AI模型:
- OpenAI GPT-4/GPT-3.5
- DeepSeek Chat
- 本地模型接口

作者: AI机械臂控制系统
版本: 2.0.0
"""

import os
import json
import re
import yaml
import logging
from typing import Dict, List, Optional, Tuple, Union
from dataclasses import dataclass, asdict
from datetime import datetime
from enum import Enum
import uuid

try:
    import openai
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False
    logging.warning("OpenAI library not available")

try:
    import requests
    REQUESTS_AVAILABLE = True
except ImportError:
    REQUESTS_AVAILABLE = False
    logging.warning("Requests library not available")

try:
    from pydantic import BaseModel, Field, validator
    PYDANTIC_AVAILABLE = True
except ImportError:
    PYDANTIC_AVAILABLE = False
    logging.warning("Pydantic library not available")


class CommandType(Enum):
    """命令类型枚举"""
    MOVE = "move"           # 移动指令
    PICK = "pick"           # 抓取指令  
    PLACE = "place"         # 放置指令
    STOP = "stop"           # 停止指令
    HOME = "home"           # 回家指令
    WAIT = "wait"           # 等待指令
    SPEED = "speed"         # 速度调整
    ROTATE = "rotate"       # 旋转指令
    OPEN_GRIPPER = "open_gripper"   # 张开夹爪
    CLOSE_GRIPPER = "close_gripper" # 闭合夹爪


class MotionType(Enum):
    """运动类型枚举"""
    LINEAR = "linear"       # 直线运动
    JOINT = "joint"         # 关节运动
    CIRCULAR = "circular"   # 圆弧运动
    SPLINE = "spline"       # 样条曲线


class CoordinateSystem(Enum):
    """坐标系类型"""
    WORLD = "world"         # 世界坐标系
    BASE = "base"           # 基座坐标系
    TOOL = "tool"           # 工具坐标系
    USER = "user"           # 用户坐标系


@dataclass
class Position:
    """位置信息"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    def to_list(self) -> List[float]:
        return [self.x, self.y, self.z]
    
    def is_valid(self) -> bool:
        """检查位置是否有效"""
        return all(isinstance(coord, (int, float)) for coord in [self.x, self.y, self.z])


@dataclass  
class Orientation:
    """姿态信息（四元数）"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0
    
    def to_list(self) -> List[float]:
        return [self.x, self.y, self.z, self.w]
    
    def is_valid(self) -> bool:
        """检查姿态是否有效"""
        return all(isinstance(coord, (int, float)) for coord in [self.x, self.y, self.z, self.w])


@dataclass
class MotionParameters:
    """运动参数"""
    speed: float = 0.5          # 速度 (0.0-1.0)
    acceleration: float = 0.3   # 加速度 (0.0-1.0)
    deceleration: float = 0.3   # 减速度 (0.0-1.0)
    jerk: float = 0.5          # 加加速度 (0.0-1.0)
    blend_radius: float = 0.0   # 融合半径
    
    def validate(self) -> bool:
        """验证参数范围"""
        return all(0.0 <= param <= 1.0 for param in [
            self.speed, self.acceleration, self.deceleration, self.jerk
        ])


@dataclass
class GripperParameters:
    """夹爪参数"""
    action: Optional[bool] = None  # True=夹取, False=释放, None=不变
    force: float = 0.5            # 力度 (0.0-1.0)
    position: float = 0.5         # 开口度 (0.0-1.0)
    speed: float = 0.5            # 动作速度 (0.0-1.0)


@dataclass
class ParsedCommand:
    """解析后的完整命令"""
    # 基本信息
    command_id: str
    command_type: CommandType
    timestamp: str
    
    # 位置和姿态
    target_position: Optional[Position] = None
    target_orientation: Optional[Orientation] = None
    coordinate_system: CoordinateSystem = CoordinateSystem.WORLD
    
    # 运动参数
    motion_type: MotionType = MotionType.LINEAR
    motion_params: MotionParameters = None
    
    # 夹爪参数
    gripper_params: GripperParameters = None
    
    # 上下文信息
    original_text: str = ""
    extracted_objects: List[str] = None
    confidence: float = 0.0
    
    # 可选参数
    wait_time: float = 0.0        # 等待时间（秒）
    relative_movement: bool = False  # 是否为相对运动
    force_control: bool = False   # 是否启用力控制
    
    def __post_init__(self):
        if self.motion_params is None:
            self.motion_params = MotionParameters()
        if self.gripper_params is None:
            self.gripper_params = GripperParameters()
        if self.extracted_objects is None:
            self.extracted_objects = []
    
    def to_dict(self) -> Dict:
        """转换为字典格式"""
        return asdict(self)
    
    def to_json(self) -> str:
        """转换为JSON字符串"""
        def convert_enum(obj):
            if isinstance(obj, Enum):
                return obj.value
            return obj
            
        data = asdict(self)
        
        # 转换枚举类型
        for key, value in data.items():
            if isinstance(value, Enum):
                data[key] = value.value
        
        return json.dumps(data, ensure_ascii=False, indent=2, default=convert_enum)


class AIProvider:
    """AI提供商基类"""
    
    def __init__(self, config: Dict):
        self.config = config
        self.model_name = config.get('model', 'unknown')
    
    def generate_response(self, prompt: str, system_prompt: str) -> str:
        """生成AI响应"""
        raise NotImplementedError
    
    def is_available(self) -> bool:
        """检查提供商是否可用"""
        raise NotImplementedError


class OpenAIProvider(AIProvider):
    """OpenAI API提供商"""
    
    def __init__(self, config: Dict):
        super().__init__(config)
        if not OPENAI_AVAILABLE:
            raise ImportError("OpenAI library not installed")
        
        self.client = openai.OpenAI(
            api_key=config.get('api_key') or os.getenv('OPENAI_API_KEY')
        )
        self.model = config.get('model', 'gpt-4')
        self.temperature = config.get('temperature', 0.1)
        self.max_tokens = config.get('max_tokens', 2000)
    
    def generate_response(self, prompt: str, system_prompt: str) -> str:
        """调用OpenAI API"""
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": prompt}
                ],
                temperature=self.temperature,
                max_tokens=self.max_tokens
            )
            return response.choices[0].message.content.strip()
        except Exception as e:
            raise Exception(f"OpenAI API调用失败: {e}")
    
    def is_available(self) -> bool:
        return OPENAI_AVAILABLE and self.client is not None


class DeepSeekProvider(AIProvider):
    """DeepSeek API提供商"""
    
    def __init__(self, config: Dict):
        super().__init__(config)
        if not OPENAI_AVAILABLE:
            raise ImportError("OpenAI library not installed for DeepSeek")
        
        self.client = openai.OpenAI(
            api_key=config.get('api_key') or os.getenv('DEEPSEEK_API_KEY'),
            base_url=config.get('base_url', 'https://api.deepseek.com')
        )
        self.model = config.get('model', 'deepseek-chat')
        self.temperature = config.get('temperature', 0.1)
        self.max_tokens = config.get('max_tokens', 2000)
    
    def generate_response(self, prompt: str, system_prompt: str) -> str:
        """调用DeepSeek API"""
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": prompt}
                ],
                temperature=self.temperature,
                max_tokens=self.max_tokens
            )
            return response.choices[0].message.content.strip()
        except Exception as e:
            raise Exception(f"DeepSeek API调用失败: {e}")
    
    def is_available(self) -> bool:
        return OPENAI_AVAILABLE and self.client is not None


class LocalModelProvider(AIProvider):
    """本地模型提供商（通过HTTP API）"""
    
    def __init__(self, config: Dict):
        super().__init__(config)
        if not REQUESTS_AVAILABLE:
            raise ImportError("Requests library not installed")
        
        self.base_url = config.get('base_url', 'http://localhost:8000')
        self.model = config.get('model', 'local-chat')
        self.timeout = config.get('timeout', 30)
    
    def generate_response(self, prompt: str, system_prompt: str) -> str:
        """调用本地模型API"""
        try:
            payload = {
                "model": self.model,
                "messages": [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": prompt}
                ],
                "temperature": self.config.get('temperature', 0.1),
                "max_tokens": self.config.get('max_tokens', 2000)
            }
            
            response = requests.post(
                f"{self.base_url}/v1/chat/completions",
                json=payload,
                timeout=self.timeout
            )
            response.raise_for_status()
            
            result = response.json()
            return result['choices'][0]['message']['content'].strip()
            
        except Exception as e:
            raise Exception(f"本地模型API调用失败: {e}")
    
    def is_available(self) -> bool:
        try:
            response = requests.get(f"{self.base_url}/health", timeout=5)
            return response.status_code == 200
        except:
            return False


class Text2CmdParser:
    """Text2Cmd核心解析器"""
    
    def __init__(self, config_path: str = None):
        """
        初始化解析器
        Args:
            config_path: 配置文件路径
        """
        self.logger = self._setup_logger()
        self.config = self._load_config(config_path)
        self.ai_provider = self._setup_ai_provider()
        self.system_prompt = self._build_system_prompt()
        
        # 预定义词典
        self.position_keywords = self._load_position_keywords()
        self.speed_keywords = self._load_speed_keywords()
        self.object_keywords = self._load_object_keywords()
        
        # 工作空间限制
        self.workspace_limits = self.config.get('workspace_limits', {
            'x': [-50.0, 50.0], 'y': [-50.0, 50.0], 'z': [0.0, 50.0]
        })
        
        self.logger.info(f"Text2Cmd解析器初始化完成，使用AI提供商: {self.ai_provider.__class__.__name__}")
    
    def _setup_logger(self) -> logging.Logger:
        """设置日志"""
        logger = logging.getLogger('Text2Cmd')
        logger.setLevel(logging.INFO)
        
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        
        return logger
    
    def _load_config(self, config_path: str) -> Dict:
        """加载配置文件"""
        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    if config_path.endswith('.yaml') or config_path.endswith('.yml'):
                        return yaml.safe_load(f)
                    else:
                        return json.load(f)
            except Exception as e:
                self.logger.warning(f"配置文件加载失败: {e}，使用默认配置")
        
        # 默认配置
        return {
            'ai_provider': 'deepseek',
            'ai_configs': {
                'openai': {
                    'model': 'gpt-4',
                    'temperature': 0.1,
                    'max_tokens': 2000
                },
                'deepseek': {
                    'model': 'deepseek-chat',
                    'temperature': 0.1,
                    'max_tokens': 2000
                },
                'local': {
                    'base_url': 'http://localhost:8000',
                    'model': 'local-chat',
                    'temperature': 0.1,
                    'max_tokens': 2000
                }
            }
        }
    
    def _setup_ai_provider(self) -> AIProvider:
        """设置AI提供商"""
        provider_name = self.config.get('ai_provider', 'deepseek')
        ai_configs = self.config.get('ai_configs', {})
        
        try:
            if provider_name == 'openai':
                return OpenAIProvider(ai_configs.get('openai', {}))
            elif provider_name == 'deepseek':
                return DeepSeekProvider(ai_configs.get('deepseek', {}))
            elif provider_name == 'local':
                return LocalModelProvider(ai_configs.get('local', {}))
            else:
                self.logger.warning(f"未知的AI提供商: {provider_name}，使用DeepSeek")
                return DeepSeekProvider(ai_configs.get('deepseek', {}))
        except Exception as e:
            self.logger.error(f"AI提供商初始化失败: {e}")
            raise
    
    def _build_system_prompt(self) -> str:
        """构建系统提示词"""
        return """你是一个专业的机械臂控制指令解析专家。你需要将中文自然语言指令精确解析为结构化的机械臂控制命令。

**核心任务：**
将用户的中文指令转换为JSON格式的机械臂控制命令，包含所有必要的参数。

**支持的命令类型：**
- move: 移动到指定位置
- pick: 抓取物体
- place: 放置物体
- stop: 停止运动
- home: 回到初始位置
- wait: 等待指定时间
- speed: 调整运动速度
- rotate: 旋转到指定角度
- open_gripper: 张开夹爪
- close_gripper: 闭合夹爪

**运动类型：**
- linear: 直线运动
- joint: 关节运动
- circular: 圆弧运动
- spline: 样条曲线

**坐标系：**
- world: 世界坐标系（默认）
- base: 基座坐标系
- tool: 工具坐标系
- user: 用户坐标系

**解析规则：**
1. 提取动作类型和目标位置
2. 识别速度关键词（慢=0.3, 正常=0.5, 快=0.7）
3. 检测物体名称
4. 分析运动方式和坐标系
5. 设置合理的默认参数

**输出格式要求：**
严格按照以下JSON结构输出，不要添加任何额外文字：

```json
{
    "command_type": "move",
    "target_position": {"x": 5.0, "y": 1.0, "z": 10.0},
    "target_orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "coordinate_system": "world",
    "motion_type": "linear",
    "motion_params": {
        "speed": 0.5,
        "acceleration": 0.3,
        "deceleration": 0.3,
        "jerk": 0.5,
        "blend_radius": 0.0
    },
    "gripper_params": {
        "action": null,
        "force": 0.5,
        "position": 0.5,
        "speed": 0.5
    },
    "extracted_objects": [],
    "confidence": 0.95,
    "wait_time": 0.0,
    "relative_movement": false,
    "force_control": false
}
```

**示例解析：**
输入："缓慢地移动到坐标5,1,10"
输出：上述JSON格式，其中speed设为0.3，target_position为{x:5.0, y:1.0, z:10.0}

现在请解析以下指令："""
    
    def _load_position_keywords(self) -> Dict[str, List[float]]:
        """加载位置关键词字典"""
        return {
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
    
    def _load_speed_keywords(self) -> Dict[str, float]:
        """加载速度关键词字典"""
        return {
            "极慢": 0.1, "非常慢": 0.15, "很慢": 0.2, "慢": 0.3, "缓慢": 0.3,
            "慢速": 0.3, "低速": 0.3, "正常": 0.5, "一般": 0.5, "普通": 0.5,
            "中速": 0.5, "标准": 0.5, "快": 0.7, "快速": 0.7, "高速": 0.7,
            "很快": 0.8, "非常快": 0.9, "极快": 0.95, "最快": 1.0, "全速": 1.0
        }
    
    def _load_object_keywords(self) -> List[str]:
        """加载物体关键词列表"""
        return [
            "苹果", "香蕉", "橙子", "杯子", "瓶子", "盒子", "球", "积木",
            "螺丝", "零件", "工具", "物体", "物件", "东西", "货物", "产品",
            "元件", "组件", "材料", "样品", "试件", "模型", "玩具", "器具"
        ]
    
    def parse(self, text: str) -> ParsedCommand:
        """
        解析文本指令
        Args:
            text: 中文指令文本
        Returns:
            ParsedCommand: 解析后的命令对象
        """
        try:
            self.logger.info(f"开始解析指令: {text}")
            
            # 生成唯一命令ID
            command_id = str(uuid.uuid4())
            timestamp = datetime.now().isoformat()
            
            # 预处理文本
            cleaned_text = self._preprocess_text(text)
            
            # 尝试规则解析（快速响应）
            rule_result = self._rule_based_parse(cleaned_text)
            if rule_result and rule_result.confidence > 0.7:
                rule_result.command_id = command_id
                rule_result.timestamp = timestamp
                rule_result.original_text = text
                self.logger.info(f"规则解析成功: {rule_result.command_type}")
                return rule_result
            
            # AI解析
            ai_result = self._ai_parse(cleaned_text)
            if ai_result:
                ai_result.command_id = command_id
                ai_result.timestamp = timestamp
                ai_result.original_text = text
                self.logger.info(f"AI解析成功: {ai_result.command_type}")
                return ai_result
            
            # 后备解析
            fallback_result = self._fallback_parse(cleaned_text)
            fallback_result.command_id = command_id
            fallback_result.timestamp = timestamp
            fallback_result.original_text = text
            self.logger.warning("使用后备解析")
            return fallback_result
            
        except Exception as e:
            self.logger.error(f"指令解析失败: {e}")
            return self._create_error_command(text, str(e))
    
    def _preprocess_text(self, text: str) -> str:
        """预处理文本"""
        # 去除多余空白
        text = re.sub(r'\s+', ' ', text.strip())
        
        # 标准化标点符号
        text = text.replace('，', ',').replace('。', '').replace('！', '')
        
        # 标准化数字格式
        text = re.sub(r'(\d+)\s*，\s*(\d+)\s*，\s*(\d+)', r'\1,\2,\3', text)
        text = re.sub(r'坐标\s*[:：]?\s*([0-9,.\-\s]+)', r'坐标\1', text)
        
        return text
    
    def _rule_based_parse(self, text: str) -> Optional[ParsedCommand]:
        """基于规则的快速解析"""
        try:
            # 命令类型识别
            command_type = self._identify_command_type(text)
            
            # 位置提取
            position = self._extract_position(text)
            
            # 速度提取
            speed = self._extract_speed(text)
            
            # 物体提取
            objects = self._extract_objects(text)
            
            # 构建命令
            if command_type and position:
                return ParsedCommand(
                    command_id="",
                    command_type=command_type,
                    timestamp="",
                    target_position=position,
                    motion_params=MotionParameters(speed=speed),
                    extracted_objects=objects,
                    confidence=0.8
                )
            
            return None
            
        except Exception as e:
            self.logger.warning(f"规则解析失败: {e}")
            return None
    
    def _identify_command_type(self, text: str) -> Optional[CommandType]:
        """识别命令类型"""
        # 停止命令
        if any(word in text for word in ["停止", "暂停", "停", "stop"]):
            return CommandType.STOP
        
        # 回家命令  
        if any(word in text for word in ["回家", "回到家", "初始位置", "起始位置", "home"]):
            return CommandType.HOME
        
        # 抓取命令
        if any(word in text for word in ["抓取", "抓", "拿", "夹", "取", "pick"]):
            return CommandType.PICK
        
        # 放置命令
        if any(word in text for word in ["放置", "放下", "放", "松开", "释放", "place"]):
            return CommandType.PLACE
        
        # 夹爪命令
        if any(word in text for word in ["张开夹爪", "打开夹爪", "张开"]):
            return CommandType.OPEN_GRIPPER
        if any(word in text for word in ["闭合夹爪", "关闭夹爪", "闭合"]):
            return CommandType.CLOSE_GRIPPER
        
        # 移动命令（默认）
        if any(word in text for word in ["移动", "移到", "去", "到", "move"]):
            return CommandType.MOVE
        
        return CommandType.MOVE  # 默认为移动
    
    def _extract_position(self, text: str) -> Optional[Position]:
        """提取位置信息"""
        # 检查预定义位置
        for keyword, pos in self.position_keywords.items():
            if keyword in text:
                return Position(x=pos[0], y=pos[1], z=pos[2])
        
        # 提取数字坐标
        coord_patterns = [
            r'坐标\s*([0-9.\-]+)\s*[,，]\s*([0-9.\-]+)\s*[,，]\s*([0-9.\-]+)',
            r'位置\s*([0-9.\-]+)\s*[,，]\s*([0-9.\-]+)\s*[,，]\s*([0-9.\-]+)',
            r'([0-9.\-]+)\s*[,，]\s*([0-9.\-]+)\s*[,，]\s*([0-9.\-]+)',
        ]
        
        for pattern in coord_patterns:
            match = re.search(pattern, text)
            if match:
                try:
                    x, y, z = map(float, match.groups())
                    return Position(x=x, y=y, z=z)
                except ValueError:
                    continue
        
        return None
    
    def _extract_speed(self, text: str) -> float:
        """提取速度信息"""
        for keyword, speed in self.speed_keywords.items():
            if keyword in text:
                return speed
        return 0.5  # 默认速度
    
    def _extract_objects(self, text: str) -> List[str]:
        """提取物体信息"""
        objects = []
        for obj in self.object_keywords:
            if obj in text:
                objects.append(obj)
        
        # 提取颜色修饰的物体
        color_pattern = r'([红绿蓝黄黑白橙紫粉灰棕]色?的?)([^，。！？\s]+)'
        matches = re.findall(color_pattern, text)
        for color, obj in matches:
            objects.append(f"{color}{obj}")
        
        return list(set(objects))  # 去重
    
    def _ai_parse(self, text: str) -> Optional[ParsedCommand]:
        """AI解析"""
        try:
            if not self.ai_provider.is_available():
                self.logger.warning("AI提供商不可用")
                return None
            
            # 调用AI模型
            response = self.ai_provider.generate_response(text, self.system_prompt)
            
            # 解析AI响应
            return self._parse_ai_response(response)
            
        except Exception as e:
            self.logger.error(f"AI解析失败: {e}")
            return None
    
    def _parse_ai_response(self, response: str) -> Optional[ParsedCommand]:
        """解析AI响应"""
        try:
            # 提取JSON部分
            json_match = re.search(r'```json\s*(\{.*?\})\s*```', response, re.DOTALL)
            if json_match:
                json_str = json_match.group(1)
            else:
                # 尝试直接解析
                json_str = response.strip()
                if not json_str.startswith('{'):
                    start = json_str.find('{')
                    if start != -1:
                        json_str = json_str[start:]
                        end = json_str.rfind('}')
                        if end != -1:
                            json_str = json_str[:end+1]
            
            # 解析JSON
            data = json.loads(json_str)
            
            # 构建ParsedCommand对象
            return self._build_command_from_dict(data)
            
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON解析失败: {e}")
            return None
        except Exception as e:
            self.logger.error(f"AI响应解析失败: {e}")
            return None
    
    def _build_command_from_dict(self, data: Dict) -> ParsedCommand:
        """从字典构建命令对象"""
        # 提取基本信息
        command_type = CommandType(data.get('command_type', 'move'))
        
        # 提取位置
        pos_data = data.get('target_position')
        position = None
        if pos_data:
            position = Position(
                x=pos_data.get('x', 0.0),
                y=pos_data.get('y', 0.0), 
                z=pos_data.get('z', 0.0)
            )
        
        # 提取姿态
        ori_data = data.get('target_orientation')
        orientation = None
        if ori_data:
            orientation = Orientation(
                x=ori_data.get('x', 0.0),
                y=ori_data.get('y', 0.0),
                z=ori_data.get('z', 0.0),
                w=ori_data.get('w', 1.0)
            )
        
        # 提取运动参数
        motion_data = data.get('motion_params', {})
        motion_params = MotionParameters(
            speed=motion_data.get('speed', 0.5),
            acceleration=motion_data.get('acceleration', 0.3),
            deceleration=motion_data.get('deceleration', 0.3),
            jerk=motion_data.get('jerk', 0.5),
            blend_radius=motion_data.get('blend_radius', 0.0)
        )
        
        # 提取夹爪参数
        gripper_data = data.get('gripper_params', {})
        gripper_params = GripperParameters(
            action=gripper_data.get('action'),
            force=gripper_data.get('force', 0.5),
            position=gripper_data.get('position', 0.5),
            speed=gripper_data.get('speed', 0.5)
        )
        
        # 应用工作空间限制
        if position:
            position = self._apply_workspace_limits(position)
        
        return ParsedCommand(
            command_id="",
            command_type=command_type,
            timestamp="",
            target_position=position,
            target_orientation=orientation,
            coordinate_system=CoordinateSystem(data.get('coordinate_system', 'world')),
            motion_type=MotionType(data.get('motion_type', 'linear')),
            motion_params=motion_params,
            gripper_params=gripper_params,
            extracted_objects=data.get('extracted_objects', []),
            confidence=data.get('confidence', 0.8),
            wait_time=data.get('wait_time', 0.0),
            relative_movement=data.get('relative_movement', False),
            force_control=data.get('force_control', False)
        )
    
    def _apply_workspace_limits(self, position: Position) -> Position:
        """应用工作空间限制"""
        limits = self.workspace_limits
        
        x = max(limits['x'][0], min(limits['x'][1], position.x))
        y = max(limits['y'][0], min(limits['y'][1], position.y))
        z = max(limits['z'][0], min(limits['z'][1], position.z))
        
        if (x != position.x or y != position.y or z != position.z):
            self.logger.warning(f"位置超出工作空间，已调整: ({position.x},{position.y},{position.z}) -> ({x},{y},{z})")
        
        return Position(x=x, y=y, z=z)
    
    def _fallback_parse(self, text: str) -> ParsedCommand:
        """后备解析方法"""
        self.logger.info("使用后备解析方法")
        
        command_type = self._identify_command_type(text) or CommandType.STOP
        position = self._extract_position(text) or Position(0.0, 0.0, 0.0)
        speed = self._extract_speed(text)
        objects = self._extract_objects(text)
        
        return ParsedCommand(
            command_id="",
            command_type=command_type,
            timestamp="",
            target_position=position,
            motion_params=MotionParameters(speed=speed),
            extracted_objects=objects,
            confidence=0.5
        )
    
    def _create_error_command(self, text: str, error: str) -> ParsedCommand:
        """创建错误命令"""
        return ParsedCommand(
            command_id=str(uuid.uuid4()),
            command_type=CommandType.STOP,
            timestamp=datetime.now().isoformat(),
            original_text=text,
            confidence=0.0
        )
    
    def batch_parse(self, texts: List[str]) -> List[ParsedCommand]:
        """批量解析"""
        results = []
        for text in texts:
            result = self.parse(text)
            results.append(result)
        return results
    
    def validate_command(self, command: ParsedCommand) -> Tuple[bool, str]:
        """验证命令的有效性"""
        try:
            # 检查位置有效性
            if command.target_position and not command.target_position.is_valid():
                return False, "目标位置无效"
            
            # 检查运动参数
            if not command.motion_params.validate():
                return False, "运动参数超出范围"
            
            # 检查置信度
            if command.confidence < 0.3:
                return False, "解析置信度过低"
            
            return True, "命令有效"
            
        except Exception as e:
            return False, f"验证失败: {e}"


def create_parser(config_path: str = None) -> Text2CmdParser:
    """创建解析器实例"""
    return Text2CmdParser(config_path)


# 测试函数
def test_parser():
    """测试解析器"""
    parser = create_parser()
    
    test_commands = [
        "缓慢地移动到坐标5，1，10",
        "快速移动到坐标-10,20,15",
        "抓取红色的苹果",
        "将物体放置到放置位置",
        "回到初始位置",
        "停止所有动作",
        "张开夹爪",
        "闭合夹爪力度设为0.8",
        "等待3秒钟",
        "以正常速度圆弧运动到观察位置"
    ]
    
    print("=== Text2Cmd解析器测试 ===\n")
    
    for i, cmd in enumerate(test_commands, 1):
        print(f"测试 {i}: {cmd}")
        result = parser.parse(cmd)
        print(f"结果: {result.command_type.value} -> {result.target_position}")
        print(f"置信度: {result.confidence:.2f}")
        print(f"速度: {result.motion_params.speed}")
        print(f"物体: {result.extracted_objects}")
        print("-" * 50)


if __name__ == "__main__":
    test_parser()
