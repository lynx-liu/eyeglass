# 镜片轮廓检测操作说明

### 1. 图片输入

开始菜单-->运行-->cmd-->回车, 输入程序名称, 带图片文件名为参数, 如:

```c++
eyeglass test.jpg
```

如下图所示:

![Main](screenshot\Main.png)

### 2 参数调节

在右下角有[设置]面板, 上面有两个滑条,拖动滑条到不同的位置, 可以调整不同的参数, 以适应不同的图像环境中达到自动查找轮廓的效果:

![Setting](screenshot\Setting.png)

### 3 查找内部边缘

当前边缘检测达到要求后, 点击[FindNext]查找内轮廓：

![FindNext](screenshot\FindNext.png)

查找内轮廓时表示外轮廓已确定, 外轮廓会显示为绿色线条(未确认前是由蓝色线条串连红色的点组成):

![InnerContour](screenshot\InnerContour.png)

### 4 轮廓编辑

#### 4.1 轮廓选择

在图像区域内按下鼠标左键, 拖选, 会形成一个矩形的选框, 选框内的轮廓部分表示被选中, 如下图:

![Select](screenshot\Select.png)

#### 4.2 轮廓移动

按[上] [下] [左] [右]键可移动选框中的轮廓, 每按一下移动一像素, 按住不放, 可以持续移动.

![Move](screenshot\Move.png)

#### 4.3 轮廓删除

按[DEL]键,可删除选区中的轮廓, 如下图效果:

![Delete](screenshot\Delete.png)

#### 4.4 取消选区

在图像区域内, 单击鼠标可以取消选择, 如下图:

![Click](screenshot\Click.png)

#### 4.5 插入轮廓点

将鼠标光标移动到图像区域内, 按[i]字母键, 可以当前位置插入一个轮廓点, 如下图:

![Insert](screenshot\Insert.png)

### 5 导出轮廓

点[设置]面板中的[Save]按钮, 将所有轮廓保存到当前目录<eyeglass.dxf>文件中, 可用CAD软件打开查看, 如下图:

![dxf](screenshot\dxf.png)

