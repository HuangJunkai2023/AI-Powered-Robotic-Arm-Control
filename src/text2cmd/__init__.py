#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Text2Cmd模块 - AI大模型指令解析
"""

__version__ = "2.0.0"
__author__ = "AI机械臂控制系统"
__description__ = "将中文自然语言指令解析为结构化的机械臂控制命令"

# 导入主要接口
try:
    from .src.text2cmd_api import parse_command, parse_commands, get_api
    from .src.text2cmd_parser import Text2CmdParser, ParsedCommand
    
    __all__ = [
        'parse_command',
        'parse_commands', 
        'get_api',
        'Text2CmdParser',
        'ParsedCommand'
    ]
except ImportError:
    # 处理相对导入失败的情况
    __all__ = []

print(f"Text2Cmd模块 v{__version__} 已加载")
