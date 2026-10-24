import os
import torch
from ultralytics import YOLO

# ============================================================
# 第七个项目 · 训练 YOLOv8n
# 前提：已经运行 yolo_step4_dataset.py 生成 yolo_lesson/dataset
# 输出：yolo_lesson/runs/full30/weights/best.pt
# ============================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
data_yaml = os.path.join(BASE_DIR, "yolo_lesson", "dataset", "data.yaml")
if not os.path.exists(data_yaml):
    raise SystemExit("找不到数据集，请先运行: python yolo_step4_dataset.py")

device = 0 if torch.cuda.is_available() else "cpu"
print("使用设备:", device)
if device == "cpu":
    print("警告：当前 PyTorch 是 CPU 版，训练会慢很多。")

# 如果本目录有 yolov8n.pt 就用本地的；否则 Ultralytics 会自动下载
model = YOLO(os.path.join(BASE_DIR, "yolov8n.pt")
             if os.path.exists(os.path.join(BASE_DIR, "yolov8n.pt"))
             else "yolov8n.pt")

model.train(
    data=data_yaml,
    epochs=30,
    patience=10,       # 验证指标 10 轮不提升就早停
    imgsz=640,
    batch=16,
    device=device,
    workers=0,         # Windows 下避免 dataloader 卡住
    amp=False,         # 避开 AMP 检查下载新模型的问题
    project=os.path.abspath(os.path.join(BASE_DIR, "yolo_lesson", "runs")),
    name="full30",
    exist_ok=True,
    plots=True,
    verbose=True,
)
