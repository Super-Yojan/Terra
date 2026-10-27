# yolo26n-rover

YOLO26n detector trained on three classes: `ball`, `blue bucket`, `orange bucket`
(100 epochs, 640 px, val mAP50 0.95, precision 0.95, recall 0.93; see `results.csv`).

TerraPhone runs it as `mobile/ios/TerraPhone/Models/RoverDetector.mlpackage` for the
L4 target search and the bucket/ball object map. The phone compiles the package on
first launch and caches it in Application Support.

## Re-export to Core ML

```sh
pip install ultralytics coremltools
yolo export model=weights/best.pt format=coreml nms=False half=True imgsz=640
mv weights/best.mlpackage ../../mobile/ios/TerraPhone/Models/RoverDetector.mlpackage
```

`nms=False` keeps YOLO26's end-to-end head: one output of shape `[1, 300, 6]`, each row
`[x1, y1, x2, y2, score, class]` in 640 px input coordinates, with duplicates already
removed. `ObjectSearchDetector.swift` reads that layout and Vision letterboxes the
camera image (`.scaleFit`) the same way Ultralytics does.

The exported package was made on Linux with ultralytics 8.4.190, coremltools 9.0,
torch 2.7.0 and numpy 2.5. That coremltools fails on numpy 2.5 in its `int` cast with
"only 0-dimensional arrays can be converted to Python scalars"; changing
`dtype(x.val)` to `dtype(x.val.item())` in `_cast` in
`coremltools/converters/mil/frontend/torch/ops.py` fixes it.
