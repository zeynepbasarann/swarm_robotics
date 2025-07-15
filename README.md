# 🧠🤖 Swarm Robotics with ROS2 & CoppeliaSim

Bu proje, **ROS2 + CoppeliaSim** simülasyon ortamında çalışan yapay zekâ destekli çoklu mobil robot sistemini simüle eder. Robotlar **sürü davranışı**, **görev paylaşımı** ve **engelden kaçınma** yetenekleri kazanarak hedefe yönelir.

## 🚀 Proje Özellikleri

- 🧠 PPO tabanlı Deep Reinforcement Learning (Stable-Baselines3)
- 🧭 Çok robotlu hedef takibi ve görev devretme
- ⚠️ Çarpışmadan kaçınma (ultrasonik sensörlerle)
- 🔄 ROS 2 tabanlı gerçek zamanlı iletişim
- 🌐 Simülasyon: CoppeliaSim + simExtROS2Interface


## 🧩 Gereksinimler

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- [CoppeliaSim v4.1.0](https://www.coppeliarobotics.com/)
- Stable-Baselines3 (`pip install stable-baselines3[extra]`)
- simExtROS2Interface (alt modül olarak eklenmiştir)

## 🔧 Kurulum

```bash
git clone --recurse-submodules https://github.com/zeynepbasarann/swarm_robotics.git
cd swarm_robotics
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```
## 🎮 Simülasyonu Başlatma

CoppeliaSim içinde sahneyi aç:

        sim/scene.ttt

ROS2 düğümlerini başlat:

    source /opt/ros/humble/setup.bash
    source venv/bin/activate
    python3 python/multi_test.py

Eğitim başlatmak için:

    python3 python/multi_train.py
    

