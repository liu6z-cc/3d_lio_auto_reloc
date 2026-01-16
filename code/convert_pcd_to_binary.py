#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
PCD格式转换工具 - 优化加载速度
将PCD文件转换为binary格式（未压缩），加载速度更快
虽然文件会变大，但加载速度可提升3-5倍

使用方法:
    python convert_pcd_to_binary.py [输入文件] [输出文件]
"""

import sys
import os

# 检查是否已安装open3d
try:
    import open3d as o3d
except ImportError:
    print("错误: 需要安装open3d库")
    print("请运行: pip install open3d-python")
    sys.exit(1)

def convert_to_binary(input_file, output_file):
    """
    转换PCD为binary格式
    """
    print("Loading {}...".format(input_file))
    try:
        pcd = o3d.io.read_point_cloud(input_file)
    except Exception as e:
        print("读取文件失败: {}".format(e))
        return
    
    points = len(pcd.points)
    print("Total points: {:,}".format(points))
    
    print("Saving to binary format: {}...".format(output_file))
    
    try:
        # write_ascii=False, compressed=False -> binary格式
        # 注意: Python2的open3d可能需要不同的参数名
        # 一些版本的open3d使用write_ascii参数
        o3d.io.write_point_cloud(output_file, pcd, write_ascii=False, compressed=False)
    except TypeError:
        # 如果参数名不同，尝试其他写法
        try:
            o3d.io.write_point_cloud(output_file, pcd, write_ascii=False)
        except Exception as e:
            print("保存文件失败: {}".format(e))
            return
    
    # 显示文件大小
    input_size = os.path.getsize(input_file) / (1024.0 * 1024.0)
    output_size = os.path.getsize(output_file) / (1024.0 * 1024.0)
    print("\nFile size: {:.1f}MB -> {:.1f}MB".format(input_size, output_size))
    print("Size increase: {:.1f}MB ({:.1f}%)".format(
        output_size - input_size, 
        (output_size/input_size - 1) * 100
    ))
    print("\n注意: 文件虽然变大，但加载速度会快3-5倍！")
    print("Done!")

if __name__ == "__main__":
    default_input = "suo_scans.pcd"
    default_output = "suo_scans_binary.pcd"
    
    if len(sys.argv) >= 2:
        input_file = sys.argv[1]
    else:
        input_file = default_input
    
    if len(sys.argv) >= 3:
        output_file = sys.argv[2]
    else:
        output_file = default_output
    
    # 检查输入文件是否存在
    if not os.path.exists(input_file):
        print("错误: 输入文件不存在: {}".format(input_file))
        sys.exit(1)
    
    convert_to_binary(input_file, output_file)
