from ultralytics import YOLO

model = YOLO("best.pt")
result = model.predict(source = '0', show = True)
print(result)