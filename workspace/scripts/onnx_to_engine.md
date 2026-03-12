# ONNX -> engine
* 首先进入本地tensorrt目录下  
* 使用`find ~/TensorRT-8.6.1.6 -name trtexec`查找trtexec的真实位置。  
他可能输出`/home/li/TensorRT-8.6.1.6/bin/trtexec`  
更改第二行`onnx=`后面的地址为真实onnx地址  
第三行保存地址也改一下
可以直接运行  
```
/home/li/TensorRT-8.6.1.6/bin/trtexec \
    --onnx=/home/li/camera_ws/src/camera_bridge/workspace/model_generate/yolo_dete_1_20/weights/best.onnx \
    --saveEngine=/home/li/camera_ws/src/camera_bridge/workspace/model_generate/yolo_dete_1_20/weights/best.engine \
    --fp16 \
    --workspace=4096
```
* 也可以把路径加到环境变量中  
* `echo 'export PATH=$HOME/TensorRT-8.6.1.6/bin:$PATH' >> ~/.bashrc`
* `source ~/.bashrc`
                                  