# map_sever 

## 文件结构

```
├── CHANGELOG.rst
├── CMakeLists.txt
├── include
│   └── map_server
│       └── image_loader.h
├── launch 
│   ├── rm2021-sc.launch // 2021年西交“疆来计划”场地地图、静态变换发布、rviz显示
│   └── rm2022_wc.launch // 2022年冬令营场地地图服务、静态变换发布、rviz显示
├── package.xml
├── rviz
│   └── rviz.rviz // 配置rviz
├── scripts
│   └── crop_map
├── src
│   ├── image_loader.cpp
│   ├── main.cpp
│   ├── map_saver.cpp
│   └── map_server.dox
└── test
    ├── 2022sc_image.png // 地图照片
    ├── consumer.py
    ├── rm2022wc.yaml // 地图配置文件
    ├── rtest.cpp
    ├── rtest.xml
    ├── spectrum.png
    ├── summercamp_map.png // 地图照片
    ├── summer_camp.yaml // 地图配置文件
    ├── test_constants.cpp
    ├── test_constants.h
    ├── testmap2.png
    ├── testmap2.yaml
    ├── testmap.bmp
    ├── testmap.png
    ├── testmap.yaml
    └── utest.cpp
```

## 使用说明

1. 在将场地照片导入到test文件夹中
2. 新建`.yaml`配置文件，并修改参数
   - image:照片存放的相对路径
   - resolution:每个像素对应的实际尺寸(单位:米/像素)
   - origin:原点位置([x, y, w])
3. 修改`launch`文件中启动`map_sever`的参数
4. 运行命令 `roslaunch map_server rm2022_wc.launch `

## 其他说明

launch文件的功能

- 启动地图服务功能包
- 启动rviz,可视化tf变化
- 启动`static_transform`功能包,发布tag和map的静态变换

```html
<launch>

    <!-- Run the map server -->
    <node pkg="map_server" type="map_server" name="map_saver" args="$(find map_server)/test/rm2022wc.yaml"/>  

    <!-- 在rviz中显示-->
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find map_server)/rviz/rviz.rviz" required="true" />

    <!-- use static_transform pkg to publish tf of tags and map -->
    <node pkg="static_transform" type="static_tf" name="static_tf"/>  
    
</launch>
```

