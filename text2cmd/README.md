# Text2Cmd模块 - AI大模型指令解析

## 模块概述

Text2Cmd是AI机械臂控制系统的核心模块，专门负责将中文自然语言指令解析为结构化的机械臂控制命令。该模块集成了多种AI大模型，提供高精度的指令理解和命令生成功能。

## 核心功能

- **多AI模型支持**: 集成OpenAI GPT、DeepSeek Chat、本地模型
- **中文指令理解**: 精确解析中文自然语言指令
- **结构化输出**: 生成标准化的机械臂控制命令
- **智能后备**: 规则解析与AI解析双重保障
- **实时验证**: 命令有效性检查和工作空间限制

## 文件结构

```
text2cmd/
├── __init__.py              # 模块入口
├── demo_simple.py           # 简化演示版本（推荐测试）
├── src/
│   ├── __init__.py
│   ├── text2cmd_parser.py   # 完整AI解析器
│   └── text2cmd_api.py      # API接口
├── config/
│   └── text2cmd_config.yaml # 配置文件
├── requirements.txt         # 依赖包
├── README.md               # 本文档
└── MODULE_SUMMARY.md       # 模块总结
```

## 系统架构

```
语音输入 → AI指令解析 → ROS消息发布 → 机械臂控制 → 状态反馈
```

### 核心组件

1. **AI指令解析器** (`ai_command_parser.py`)
   - 中文指令预处理
   - AI模型调用
   - 结构化命令生成

2. **ROS控制节点** (`ai_arm_controller.py`)
   - 语音指令接收
   - 消息发布和订阅
   - 状态管理

3. **自定义消息类型**
   - `ArmCommand.msg`：机械臂控制命令
   - `CommandStatus.msg`：命令执行状态

## 安装和配置

### 1. 环境要求

- Python 3.7+
- ROS Noetic (或兼容版本)
- 有效的OpenAI或DeepSeek API密钥

### 2. 依赖安装

```bash
# 安装Python依赖
pip install -r requirements.txt

# 如果使用conda环境
conda install -c conda-forge openai pydantic python-dotenv requests
```

### 3. 环境配置

```bash
# 复制环境变量模板
cp .env.example .env

# 编辑.env文件，填入你的API密钥
nano .env
```

### 4. ROS包编译

```bash
# 在catkin工作空间中编译
cd ~/catkin_ws
catkin_make

# 刷新环境
source devel/setup.bash
```

## 使用说明

### 1. 启动系统

```bash
# 启动ROS核心
roscore

# 启动AI机械臂控制器
roslaunch ai_arm_controller ai_arm_controller.launch

# 或者带测试客户端启动
roslaunch ai_arm_controller ai_arm_controller.launch test:=true
```

### 2. 发送语音指令

```bash
# 使用测试客户端
python src/test_client.py

# 或者直接发布ROS消息
rostopic pub /voice_command std_msgs/String "data: '缓慢地移动到坐标5,1,10'"
```

### 3. 监控系统状态

```bash
# 查看机械臂命令
rostopic echo /arm_command

# 查看执行状态  
rostopic echo /arm_status

# 查看反馈信息
rostopic echo /arm_feedback
```

## 支持的指令类型

### 移动指令
- "移动到坐标5,1,10"
- "缓慢地移动到坐标-10,20,15"
- "快速移动到原点"

### 抓取指令
- "抓取红色的苹果"
- "夹取桌上的物体"
- "拿起那个杯子"

### 放置指令
- "放置到放置区域"
- "将物体放下"
- "松开夹爪"

### 系统指令
- "回到初始位置"
- "停止所有动作"
- "回家"

## 配置说明

### AI模型配置 (`config/config.yaml`)

```yaml
AI_CONFIG:
  openai:
    api_key: "your_key"
    model: "gpt-4"
    temperature: 0.1
  deepseek:
    api_key: "your_key"  
    base_url: "https://api.deepseek.com"
    model: "deepseek-chat"
```

### ROS话题配置

- `/voice_command`：语音指令输入
- `/arm_command`：机械臂控制命令输出
- `/arm_status`：命令执行状态
- `/arm_feedback`：机械臂反馈信息

## API参考

### ParsedCommand类

```python
@dataclass
class ParsedCommand:
    command_type: str                    # 命令类型
    target_position: Tuple[float, float, float]  # 目标位置
    target_orientation: Optional[Tuple[float, float, float, float]]  # 目标姿态
    speed: float                         # 移动速度
    acceleration: float                  # 加速度
    motion_type: str                     # 运动类型
    gripper_action: Optional[bool]       # 夹爪动作
    gripper_force: float                 # 夹爪力度
    extracted_objects: List[str]         # 提取的物体
    confidence: float                    # 解析置信度
```

### 消息格式

#### ArmCommand消息
```
string command_type
geometry_msgs/Point target_position
geometry_msgs/Quaternion target_orientation
float32 speed
float32 acceleration
string motion_type
bool gripper_action
float32 gripper_force
string original_command
float32 confidence
string[] extracted_objects
string timestamp
```

## 开发和扩展

### 添加新的命令类型

1. 在`ai_command_parser.py`中更新系统提示词
2. 在`CommandSchema`中添加新字段
3. 更新验证逻辑

### 集成新的AI模型

1. 在`AICommandParser`类中添加新的provider
2. 实现对应的API调用逻辑
3. 更新配置文件

### 自定义消息类型

1. 在`msg/`目录下定义新的`.msg`文件
2. 更新`CMakeLists.txt`
3. 重新编译ROS包

## 故障排除

### 常见问题

1. **导入错误**：确保已安装所有Python依赖
2. **API密钥错误**：检查`.env`文件中的API密钥
3. **ROS连接问题**：确保roscore正在运行
4. **消息类型未找到**：重新编译ROS包

### 调试模式

```bash
# 启用调试模式
export DEBUG=true

# 查看详细日志
rosrun ai_arm_controller ai_arm_controller.py
```

## 贡献指南

1. Fork项目
2. 创建特性分支
3. 提交更改
4. 创建Pull Request

## 许可证

MIT License

## 联系方式

如有问题或建议，请创建Issue或联系维护者。

---

*最后更新：2025年8月20日*
