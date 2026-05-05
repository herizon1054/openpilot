#!/usr/bin/env python3
"""
Dragonpilot 一键下载并安装 Down to Ride v6 (DTRv6) 模型
使用方法: python3 install_dtrv6.py
"""

import os
import sys
import json
import shutil
import urllib.request
from datetime import datetime
import subprocess

# ==================== 配置区域 (根据你的实际情况修改) ====================
# 运行目录 (DragonPilot 模型安装目录)
INSTALL_DIR = "/data/openpilot/selfdrive/modeld/models"
# 模型库目录 (用于存放下载的模型文件)
LIBRARY_DIR = "/data/media/0/models"
# 全局模型信息文件
GLOBAL_INFO_FILE = os.path.join(LIBRARY_DIR, "info.json")

# DTRv6 模型配置 (基于你提供的链接)
BASE_URL = "https://gitlab.com/sunnypilot/public/docs.sunnypilot.ai2/-/raw/main/models/recompiled7/model-Down%20to%20Ride%20v6%20(August%2012,%202025)-71/"

MODEL_CONFIG = {
    "name": "Down to Ride v6 (August 12, 2025)-71",
    "short_name": "dtrv6",
    "files": [
        # 请根据你图片中实际存在的文件确认以下列表
        # 1. ONNX 文件
        {
            "type": "policy_onnx",
            "url": BASE_URL + "driving_policy.onnx",
            "file_name": "driving_policy_dtrv6.onnx"
        },
        {
            "type": "vision_onnx",
            "url": BASE_URL + "driving_vision.onnx",
            "file_name": "driving_vision_dtrv6.onnx"
        },
        # 2. PKL 模型文件 (注意：图片中没有标准的pkl，只有tinygrad.pkl)
        {
            "type": "policy",
            "url": BASE_URL + "driving_policy_dtrv6_tinygrad.pkl",  # 请确认此文件名存在
            "file_name": "driving_policy_dtrv6_tinygrad.pkl"
        },
        {
            "type": "vision",
            "url": BASE_URL + "driving_vision_dtrv6_tinygrad.pkl",  # 请确认此文件名存在
            "file_name": "driving_vision_dtrv6_tinygrad.pkl"
        },
        # 3. Metadata 文件
        {
            "type": "policy_metadata",
            "url": BASE_URL + "driving_policy_dtrv6_metadata.pkl",
            "file_name": "driving_policy_dtrv6_metadata.pkl"
        },
        {
            "type": "vision_metadata",
            "url": BASE_URL + "driving_vision_dtrv6_metadata.pkl",
            "file_name": "driving_vision_dtrv6_metadata.pkl"
        }
    ]
}
# ==================== 配置结束 ====================

def download_file(url, local_path):
    """下载单个文件，显示进度"""
    try:
        print(f"   📥 正在下载: {os.path.basename(local_path)}", end="", flush=True)
        with urllib.request.urlopen(url, timeout=30) as response:
            total = int(response.headers.get('Content-Length', 0))
            with open(local_path, 'wb') as f:
                dl = 0
                while True:
                    chunk = response.read(8192)
                    if not chunk:
                        break
                    dl += len(chunk)
                    f.write(chunk)
                    if total:
                        progress = int(dl/total*100)
                        print(f"\r   📥 正在下载: {os.path.basename(local_path)} [{progress}%]", end="", flush=True)
        print(f"\r   ✅ 下载完成: {os.path.basename(local_path)}")
        return True
    except Exception as e:
        print(f"\r   ❌ 下载失败: {os.path.basename(local_path)} - {e}")
        if os.path.exists(local_path):
            os.remove(local_path)
        return False

def backup_current_models():
    """备份当前正在使用的模型文件"""
    data_dir = "/data/media/0/models"
    if not os.path.exists(data_dir):
        os.makedirs(data_dir, exist_ok=True)
    
    print("📦 备份当前模型文件...")
    backups = 0
    
    # 要备份的文件列表
    backup_files = [
        ("driving_policy_tinygrad.pkl", "driving_policy_backup.pkl"),
        ("driving_vision_tinygrad.pkl", "driving_vision_backup.pkl"),
        ("driving_policy.onnx", "driving_policy_backup.onnx"),
        ("driving_vision.onnx", "driving_vision_backup.onnx"),
        ("driving_policy_metadata.pkl", "driving_policy_metadata_backup.pkl"),
        ("driving_vision_metadata.pkl", "driving_vision_metadata_backup.pkl")
    ]
    
    for src_name, dst_name in backup_files:
        src = os.path.join(INSTALL_DIR, src_name)
        dst = os.path.join(data_dir, dst_name)
        if os.path.exists(src):
            shutil.copy2(src, dst)
            print(f"   ✅ 备份: {src_name} -> {dst_name}")
            backups += 1
    
    if backups > 0:
        print(f"✅ 已备份 {backups} 个当前模型文件")
    else:
        print("⚠️  没有找到需要备份的模型文件")

def install_dtrv6_model():
    """主安装函数"""
    print("=" * 60)
    print(f"🚀 Dragonpilot DTRv6 模型一键安装器")
    print("=" * 60)
    print(f"📁 运行目录: {INSTALL_DIR}")
    print(f"📁 库目录: {LIBRARY_DIR}")
    print(f"🎯 目标模型: {MODEL_CONFIG['name']}")
    print("=" * 60)
    
    # 检查环境
    if not os.path.exists(INSTALL_DIR):
        print(f"❌ 错误: 未找到运行目录 {INSTALL_DIR}")
        print("   请确保在 Dragonpilot 根目录运行此脚本")
        return False
    
    # 创建目录
    os.makedirs(LIBRARY_DIR, exist_ok=True)
    
    # 1. 备份当前模型
    backup_current_models()
    
    # 2. 下载所有文件
    print(f"\n📥 开始下载 {len(MODEL_CONFIG['files'])} 个模型文件...")
    file_mapping = {}
    successful_downloads = 0
    
    for file_info in MODEL_CONFIG["files"]:
        url = file_info["url"]
        filename = file_info["file_name"]
        local_path = os.path.join(LIBRARY_DIR, filename)
        
        if download_file(url, local_path):
            file_mapping[file_info["type"]] = filename
            successful_downloads += 1
    
    if successful_downloads < len(MODEL_CONFIG["files"]):
        print(f"⚠️  警告: 只成功下载了 {successful_downloads}/{len(MODEL_CONFIG['files'])} 个文件")
        if input("❓ 是否继续安装? (y/n): ").lower() != 'y':
            return False
    
    if successful_downloads == 0:
        print("❌ 错误: 没有文件下载成功")
        return False
    
    # 3. 清理运行目录
    print("\n🧹 清理运行目录...")
    subprocess.run(f'git reset HEAD "{INSTALL_DIR}/*"', shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    
    for f in os.listdir(INSTALL_DIR):
        if f.startswith(("driving_", "supercombo")) and f.endswith((".pkl", ".onnx")):
            try:
                os.remove(os.path.join(INSTALL_DIR, f))
                print(f"   🗑️ 删除: {f}")
            except:
                pass
    
    # 4. 复制并重命名文件
    print("\n📦 安装模型文件...")
    
    # 文件名映射规则
    copy_operations = []
    
    # 处理 policy 文件
    if "policy" in file_mapping:
        copy_operations.append((
            file_mapping["policy"],
            "driving_policy_tinygrad.pkl"
        ))
    
    if "policy_onnx" in file_mapping:
        copy_operations.append((
            file_mapping["policy_onnx"],
            "driving_policy.onnx"
        ))
    
    if "policy_metadata" in file_mapping:
        copy_operations.append((
            file_mapping["policy_metadata"],
            "driving_policy_metadata.pkl"
        ))
    
    # 处理 vision 文件
    if "vision" in file_mapping:
        copy_operations.append((
            file_mapping["vision"],
            "driving_vision_tinygrad.pkl"
        ))
    
    if "vision_onnx" in file_mapping:
        copy_operations.append((
            file_mapping["vision_onnx"],
            "driving_vision.onnx"
        ))
    
    if "vision_metadata" in file_mapping:
        copy_operations.append((
            file_mapping["vision_metadata"],
            "driving_vision_metadata.pkl"
        ))
    
    # 执行复制
    copied_count = 0
    for src_name, dst_name in copy_operations:
        src = os.path.join(LIBRARY_DIR, src_name)
        dst = os.path.join(INSTALL_DIR, dst_name)
        
        if os.path.exists(src):
            shutil.copy2(src, dst)
            # 添加到 git
            subprocess.run(['git', 'add', '-f', dst], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
            print(f"   ✅ {src_name} -> {dst_name}")
            copied_count += 1
        else:
            print(f"   ⚠️ 缺失文件: {src_name}")
    
    if copied_count == 0:
        print("❌ 错误: 没有文件被复制")
        return False
    
    # 5. 更新全局模型信息
    print("\n📝 更新模型配置...")
    global_info = {
        "downloaded_models": [],
        "current_model": MODEL_CONFIG["short_name"],
        "current_files": file_mapping,
        "last_updated": datetime.now().isoformat()
    }
    
    if os.path.exists(GLOBAL_INFO_FILE):
        try:
            with open(GLOBAL_INFO_FILE, 'r') as f:
                existing_info = json.load(f)
                # 保留已下载模型列表
                if "downloaded_models" in existing_info:
                    # 移除同名的旧模型
                    global_info["downloaded_models"] = [
                        m for m in existing_info["downloaded_models"]
                        if m.get("short_name") != MODEL_CONFIG["short_name"]
                    ]
        except:
            pass
    
    # 添加当前模型
    global_info["downloaded_models"].append({
        "name": MODEL_CONFIG["name"],
        "short_name": MODEL_CONFIG["short_name"],
        "files": file_mapping,
        "downloaded_at": datetime.now().isoformat()
    })
    
    # 保存配置
    with open(GLOBAL_INFO_FILE, 'w') as f:
        json.dump(global_info, f, indent=2)
    
    print(f"✅ 已将 {MODEL_CONFIG['name']} 设为当前模型")
    
    # 6. 输出安装结果
    print("\n" + "=" * 60)
    print("🎉 DTRv6 模型安装完成!")
    print("=" * 60)
    print(f"📊 下载统计: {successful_downloads}/{len(MODEL_CONFIG['files'])} 个文件")
    print(f"📊 安装统计: {copied_count} 个文件已复制到运行目录")
    print(f"📁 模型文件保存在: {LIBRARY_DIR}")
    print(f"📁 运行文件在: {INSTALL_DIR}")
    print("=" * 60)
    print("⚠️ 重要: 安装完成后需要执行以下操作:")
    print("1. 清除编译缓存: scons -c")
    print("2. 重启 Dragonpilot: sudo reboot")
    print("=" * 60)
    
    return True

def main():
    """主函数"""
    print("🔧 准备安装 DTRv6 模型...")
    
    # 确认安装
    print(f"📋 将安装以下文件:")
    for i, file_info in enumerate(MODEL_CONFIG["files"], 1):
        filename = os.path.basename(file_info["url"])
        print(f"  {i}. {filename}")
    
    confirm = input("\n❓ 确认开始安装? (y/n): ").strip().lower()
    if confirm != 'y':
        print("❌ 安装已取消")
        return
    
    # 执行安装
    if install_dtrv6_model():
        # 询问是否执行后续操作
        print("\n⚡️ 是否自动执行后续操作?")
        print("1. 仅清除编译缓存 (scons -c)")
        print("2. 清除缓存并重启 (scons -c && sudo reboot)")
        print("3. 手动操作 (稍后自行处理)")
        
        choice = input("👉 请选择 (1/2/3): ").strip()
        
        if choice == '1':
            print("🧹 正在清除编译缓存...")
            os.system("scons -c")
            print("✅ 编译缓存已清除")
        elif choice == '2':
            print("🧹 正在清除编译缓存...")
            os.system("scons -c")
            print("🔄 正在重启系统...")
            os.system("sudo reboot")
    
    print("\n✨ 脚本执行完成!")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n❌ 用户中断，安装已取消")
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()