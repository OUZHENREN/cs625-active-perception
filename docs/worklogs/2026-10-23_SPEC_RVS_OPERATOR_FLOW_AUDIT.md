# 2026-10-23 SPEC — RVS 关键算子流程审查（复刻前必读）

> 目的：把 RVS 工程里真正参与定位的算子与参数抽出来，防止 ROS 侧复刻时遗漏算法细节。


### T1-R1拍摄插槽
    - T1_TCPServerReceive [TCPServerReceiveNode] {'tcp_resource_name': 'TCPServerConnectionResource'}
    - 机器人拍照位 [EmitNode] {'type': 'Pose'}
    - deep [TransformNode] {'type': 'Pose', 'invert': '0'}
    - TyCameraAccess [TyCameraAccessNode] {}
      - DirectoryName [EmitNode] {'type': 'String', 'directory': 'ImageSave_slot_near'}
      - CloudFilename [EmitNode] {'type': 'String', 'directory': ''}
      - ColorFilename [EmitNode] {'type': 'String', 'directory': ''}
      - SaveCloud [SaveNode] {'type': 'PointCloud', 'format': 'binary'}
      - SaveColor [SaveNode] {'type': 'Image'}
        - 加载标定文件depth2robot [LoadNode] {'type': 'Pose', 'directory': ''}
        - 加载标定文件rgb2robot [LoadNode] {'type': 'Pose', 'directory': ''}
        - LoadCalibFile [LoadCalibFileNode] {'calib_file': 'ty_color_calib_ps800_207000164956.txt', 'res_x': '2560', 'res_y': '1920', 'fx': '1862.946167', 'fy': '1863.190674'}
        - Emit [EmitNode] {'type': 'Pose'}
        - depth2robot [TransformNode] {'type': 'Pose', 'invert': '0'}
      - 机器人拍照位 [EmitNode] {'type': 'Pose'}
      - deep [TransformNode] {'type': 'Pose', 'invert': '0'}
      - Transform [TransformNode] {'type': 'PointCloud'}
      - SaveCloud [SaveNode] {'type': 'PointCloud', 'format': 'binary'}
      - 插槽点云匹配（需要修改模型文件） [Percipio3DMatchingNode] {'model_filename': 'Model/slot_pointcloud20260127.ply', 'estimate_normal': '1', 'targetCount': '1', 'modelRelSamplingDistance': '0.01499999966'}
      - Z向排序 [SortListNode] {'type': 'Pose', 'mode': 'LinearSort'}
      - AtList [AtListNode] {'type': 'Pose', 'mode': 'SingleByIndex', 'index': '0'}
      - 没有产品 [EmitNode] {'type': 'String', 'directory': ''}
      - 相机掉线 [EmitNode] {'type': 'String', 'directory': ''}
      - 前缀 [EmitNode] {'type': 'String', 'directory': ''}
    - 发送R1消息 [CommonTCPClientNode] {}
    - 失败消息 [CommonTCPClientNode] {}

### T2-S2消息移至弹仓拍摄位
    - T2_TCPServerReceive [TCPServerReceiveNode] {'tcp_resource_name': 'TCPServerConnectionResource'}
    - TyCameraAccess [TyCameraAccessNode] {}
      - AIInference [AIInferenceNode] {'server_port': '2024', 'model_ID': '0'}
      - ProjectMask2Cloud [ProjectMaskNode] {'type': 'MaskList'}
      - DirectoryName [EmitNode] {'type': 'String', 'directory': 'ImageSave_box_far'}
      - CloudFilename [EmitNode] {'type': 'String', 'directory': ''}
      - ColorFilename [EmitNode] {'type': 'String', 'directory': ''}
      - SaveCloud [SaveNode] {'type': 'PointCloud', 'format': 'binary'}
      - SaveColor [SaveNode] {'type': 'Image'}
    - AtList [AtListNode] {'type': 'PointCloud', 'mode': 'SingleByIndex', 'index': '0'}
        - 加载标定文件depth2robot [LoadNode] {'type': 'Pose', 'directory': ''}
        - 加载标定文件rgb2robot [LoadNode] {'type': 'Pose', 'directory': ''}
        - LoadCalibFile [LoadCalibFileNode] {'calib_file': 'ty_color_calib_ps800_207000164956.txt', 'res_x': '2560', 'res_y': '1920', 'fx': '1862.946167', 'fy': '1863.190674'}
        - Emit [EmitNode] {'type': 'Pose'}
        - depth2robot [TransformNode] {'type': 'Pose', 'invert': '0'}
      - 机器人拍照位 [EmitNode] {'type': 'Pose'}
      - deep [TransformNode] {'type': 'Pose', 'invert': '0'}
      - Transform [TransformNode] {'type': 'PointCloud'}
      - Percipio3DMatching [Percipio3DMatchingNode] {'model_filename': 'Model/box.ply', 'estimate_normal': '1', 'targetCount': '1', 'modelRelSamplingDistance': '0.02999999933'}
      - SaveCloud [SaveNode] {'type': 'PointCloud', 'format': 'binary'}
      - Z向排序 [SortListNode] {'type': 'Pose', 'mode': 'LinearSort'}
      - AtList [AtListNode] {'type': 'Pose', 'mode': 'SingleByIndex', 'index': '0'}
      - AtList-模板匹配中心位姿 [EmitNode] {'type': 'Pose'}
      - 实际拍摄位置 [EmitNode] {'type': 'Pose'}
      - Transform [TransformNode] {'type': 'Pose', 'invert': '0'}
      - Transform_1 [TransformNode] {'type': 'Pose', 'invert': '0'}
    - 转换数据 [EmitNode] {'type': 'Pose'}
    - Transform [TransformNode] {'type': 'Pose', 'invert': '0'}
      - 没有产品 [EmitNode] {'type': 'String', 'directory': ''}
      - 相机掉线 [EmitNode] {'type': 'String', 'directory': ''}
      - 前缀 [EmitNode] {'type': 'String', 'directory': ''}
    - 发送S2消息 [CommonTCPClientNode] {}
    - 失败消息 [CommonTCPClientNode] {}

### T2-S2消息移至弹仓拍摄位_1
    - T2_TCPServerReceive [TCPServerReceiveNode] {'tcp_resource_name': 'TCPServerConnectionResource'}
    - TyCameraAccess [TyCameraAccessNode] {}
        - 加载标定文件depth2robot [LoadNode] {'type': 'Pose', 'directory': ''}
        - 加载标定文件rgb2robot [LoadNode] {'type': 'Pose', 'directory': ''}
        - LoadCalibFile [LoadCalibFileNode] {'calib_file': 'ty_color_calib_ps800_207000164956.txt', 'res_x': '2560', 'res_y': '1920', 'fx': '1862.946167', 'fy': '1863.190674'}
        - Emit [EmitNode] {'type': 'Pose'}
        - depth2robot [TransformNode] {'type': 'Pose', 'invert': '0'}
      - 机器人拍照位 [EmitNode] {'type': 'Pose'}
      - deep [TransformNode] {'type': 'Pose', 'invert': '0'}
      - Transform [TransformNode] {'type': 'PointCloud'}
      - DirectoryName [EmitNode] {'type': 'String', 'directory': 'Model/environment_point_cloud'}
      - CloudFilename [EmitNode] {'type': 'String', 'directory': ''}
      - SaveCloud [SaveNode] {'type': 'PointCloud', 'format': 'binary'}

### T3-S3消息移至抓取位
    - T3_TCPServerReceive [TCPServerReceiveNode] {'tcp_resource_name': 'TCPServerConnectionResource'}
    - 机器人拍照位 [EmitNode] {'type': 'Pose'}
    - deep [TransformNode] {'type': 'Pose', 'invert': '0'}
    - TyCameraAccess [TyCameraAccessNode] {}
      - DirectoryName [EmitNode] {'type': 'String', 'directory': 'ImageSave_box_near'}
      - CloudFilename [EmitNode] {'type': 'String', 'directory': ''}
      - ColorFilename [EmitNode] {'type': 'String', 'directory': ''}
      - SaveCloud [SaveNode] {'type': 'PointCloud', 'format': 'binary'}
      - SaveColor [SaveNode] {'type': 'Image'}
        - 加载标定文件depth2robot [LoadNode] {'type': 'Pose', 'directory': ''}
        - 加载标定文件rgb2robot [LoadNode] {'type': 'Pose', 'directory': ''}
        - LoadCalibFile [LoadCalibFileNode] {'calib_file': 'ty_color_calib_ps800_207000164956.txt', 'res_x': '2560', 'res_y': '1920', 'fx': '1862.946167', 'fy': '1863.190674'}
        - Emit [EmitNode] {'type': 'Pose'}
        - depth2robot [TransformNode] {'type': 'Pose', 'invert': '0'}
      - 机器人拍照位 [EmitNode] {'type': 'Pose'}
      - deep [TransformNode] {'type': 'Pose', 'invert': '0'}
      - Transform [TransformNode] {'type': 'PointCloud'}
      - Percipio3DMatching [Percipio3DMatchingNode] {'model_filename': 'Model/box.ply', 'estimate_normal': '1', 'targetCount': '1', 'modelRelSamplingDistance': '0.01499999966'}
      - SaveCloud [SaveNode] {'type': 'PointCloud', 'format': 'binary'}
      - Z向排序 [SortListNode] {'type': 'Pose', 'mode': 'LinearSort'}
      - AtList [AtListNode] {'type': 'Pose', 'mode': 'SingleByIndex', 'index': '0'}
      - 没有产品 [EmitNode] {'type': 'String', 'directory': ''}
      - 相机掉线 [EmitNode] {'type': 'String', 'directory': ''}
      - 前缀 [EmitNode] {'type': 'String', 'directory': ''}
    - 发送S3消息 [CommonTCPClientNode] {}
    - 失败消息 [CommonTCPClientNode] {}

## 复刻时必须对齐的要点

1. **AI 推理**：AIInferenceNode → AI 服务（独立端口）→ 模型 ID + 分数阈值；
   输出是掩膜，不是检测框。
2. **掩膜→点云**：ProjectMaskNode 按相机内参把掩膜投影成点云，
   这是 ROS 侧 `depth_pose_once.py` 要等价实现的核心一步。
3. **3D 模板匹配**：Percipio3DMatchingNode 使用厂商 .ply 模板 +
   `modelRelSamplingDistance`（当前 0.03）+ `estimate_normal`；
   这两个参数直接决定精度与耗时，复刻时必须暴露为可配置项。
4. **坐标变换**：TransformNode + 手眼标定文件（DepthToRobotTCP / ColorToRobotTCP），
   顺序是「相机系 → 机器人系」，不能颠倒。
5. **结果选择**：SortListNode（按 Z 向排序）+ AtListNode（按索引取单个），
   对应我们 ROS 侧的"多候选里挑最近且置信最高"。
6. **回传**：CommonTCPClientNode 把结果发回任务层，字段与顺序需与现有接口一致。

## 待核对清单（下次逐项打勾）

- [ ] AI 推理的分数阈值与 RVS 中一致
- [ ] 模板采样步长 0.03 是否需要按 5 mm 简化模型调整
- [ ] 法向估计是否开启、对配准结果的影响
- [ ] 手眼变换方向与现有 TF 链一致
- [ ] 多目标时的排序/选择规则一致
- [ ] 输出位姿的坐标系与约定（是否中心点、是否含工具偏移）
