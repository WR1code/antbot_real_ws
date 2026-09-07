#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import argparse
import xml.etree.ElementTree as ET
from xml.dom import minidom

def prettify(elem):
    # 将 ElementTree 转为字符串
    rough_string = ET.tostring(elem, 'utf-8')
    # 使用 minidom 进行格式化，让缩进更漂亮
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def manage_action(input_file, output_file, waypoint_numbers, mode):
    try:
        tree = ET.parse(input_file)
        root = tree.getroot()
    except Exception as e:
        print(f"解析 XML 失败: {e}")
        return False

    # 【关键修改】改为查找小写的 'waypoint'
    waypoints = root.findall('waypoint')
    if not waypoints:
        print("错误：未找到任何 <waypoint> 元素 (请检查标签是否为全小写)")
        return False

    modified = []
    for wp in waypoints:
        # 【关键修改】从属性中获取 name
        wp_name = wp.get('name')
        if wp_name not in waypoint_numbers:
            continue

        # 【关键修改】查找小写的 'action' 标签
        action_elem = wp.find('action')
        
        if mode == 'add':
            if action_elem is not None:
                action_elem.text = 'detect' # 如果存在则修改
            else:
                action_elem = ET.SubElement(wp, 'action')
                action_elem.text = 'detect'
            modified.append(wp_name)
            print(f"已为航点 {wp_name} 设置 <action>detect</action>")
            
        elif mode == 'remove':
            if action_elem is not None:
                action_elem.text = 'none' # 按照你的示例，不删标签，而是改回 none
                modified.append(wp_name)
                print(f"已将航点 {wp_name} 的动作恢复为 <action>none</action>")
            else:
                print(f"航点 {wp_name} 本身就没有 action 标签")

    if modified:
        # 保存文件，确保不破坏原有的 pose 属性结构
        with open(output_file, 'w', encoding='utf-8') as f:
            # 这里的 prettify 会处理好嵌套的 position/orientation 属性
            xml_str = prettify(root)
            # 过滤掉 minidom 自动生成的 xml 声明头（如果不需要的话）
            if '?>' in xml_str:
                xml_str = xml_str.split('?>', 1)[1]
            f.write(xml_str.strip())
        return True
    else:
        print("未进行任何修改")
        return False

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='修改航点的 action 属性')
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--waypoints', required=True)
    parser.add_argument('--mode', choices=['add', 'remove'], default='add')
    args = parser.parse_args()

    numbers = [num.strip() for num in args.waypoints.split(',') if num.strip()]
    success = manage_action(args.input, args.output, numbers, args.mode)
    sys.exit(0 if success else 1)