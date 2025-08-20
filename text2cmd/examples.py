#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Text2Cmd模块使用示例
演示如何在其他模块中集成使用Text2Cmd
"""

import sys
import os

# 添加当前目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

def example_basic_usage():
    """基本使用示例"""
    print("=== Text2Cmd基本使用示例 ===\n")
    
    # 导入简化解析器
    from demo_simple import SimpleParser
    
    parser = SimpleParser()
    
    # 测试指令
    commands = [
        "移动到坐标5,1,10",
        "抓取红色的苹果", 
        "放置到指定位置",
        "回到初始位置"
    ]
    
    for cmd in commands:
        result = parser.parse(cmd)
        print(f"输入: {cmd}")
        print(f"输出: {result['command_type']} -> {result.get('target_position', 'N/A')}")
        print(f"置信度: {result['confidence']:.2f}")
        print("-" * 40)

def example_integration():
    """模块集成示例"""
    print("\n=== 模块集成示例 ===\n")
    
    # 模拟voice2text模块输出
    def mock_voice2text_output():
        return "缓慢地移动到坐标10,20,30"
    
    # 模拟cmd2ros2模块输入
    def mock_cmd2ros2_input(parsed_command):
        print(f"ROS消息发布: {parsed_command['command_type']}")
        if parsed_command.get('target_position'):
            pos = parsed_command['target_position']
            print(f"目标位置: ({pos['x']}, {pos['y']}, {pos['z']})")
        print(f"运动速度: {parsed_command['motion_params']['speed']}")
    
    # 模拟ros2aubo模块输入
    def mock_ros2aubo_input(parsed_command):
        aubo_format = {
            "command": parsed_command['command_type'],
            "position": parsed_command.get('target_position'),
            "velocity": parsed_command['motion_params']['speed'],
            "force": parsed_command['gripper_params']['force']
        }
        print(f"Auburn机械臂格式: {aubo_format}")
    
    # 完整流程演示
    from demo_simple import SimpleParser
    
    parser = SimpleParser()
    
    # 1. 接收voice2text输出
    voice_text = mock_voice2text_output()
    print(f"语音识别结果: {voice_text}")
    
    # 2. Text2Cmd解析
    parsed_result = parser.parse(voice_text)
    print(f"Text2Cmd解析: {parsed_result['command_type']}")
    
    # 3. 发送到cmd2ros2
    mock_cmd2ros2_input(parsed_result)
    
    # 4. 发送到ros2aubo
    mock_ros2aubo_input(parsed_result)

def example_error_handling():
    """错误处理示例"""
    print("\n=== 错误处理示例 ===\n")
    
    from demo_simple import SimpleParser
    
    parser = SimpleParser()
    
    # 测试各种边界情况
    test_cases = [
        "",  # 空输入
        "asdfghjkl",  # 无意义输入
        "移动到",  # 不完整指令
        "移动到坐标999,999,999",  # 超出工作空间
    ]
    
    for test_input in test_cases:
        try:
            result = parser.parse(test_input)
            print(f"输入: '{test_input}'")
            print(f"结果: {result['command_type']} (置信度: {result['confidence']:.2f})")
            
            # 验证结果
            if result['confidence'] < 0.3:
                print("⚠️ 置信度过低，建议重新输入")
            else:
                print("✓ 解析成功")
                
        except Exception as e:
            print(f"错误: {e}")
        
        print("-" * 30)

if __name__ == "__main__":
    # 运行所有示例
    example_basic_usage()
    example_integration() 
    example_error_handling()
    
    print("\n=== 示例完成 ===")
    print("Text2Cmd模块可以轻松集成到其他模块中！")
