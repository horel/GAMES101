# GAMES101 ![](doc/labels/图形学-GAMES101-blue.svg) ![](doc/labels/现代计算机图形学入门-闫令琪-orange.svg) ![](doc/labels/linux.svg)
仅供参考！如果你真的热爱图形学，希望可以自己先尝试写作业！
直接看代码是收获不到知识的！

Table of Contents
=================

* [参考作业实现](#参考作业实现)
   * [作业0 环境配置](#作业0-环境配置)
   * [作业1 旋转与投影](#作业1-旋转与投影)
   * [作业2 Triangles and Z-buffering](#作业2-triangles-and-z-buffering)
      * [Basic](#Basic)
      * [MSAA 4X](#msaa-4x)
   * [作业3 Pipeline and Shading](#作业3-pipeline-and-shading)
      * [Basic](#texture)
      * [Bonus](#bonus1-other-models)
   * [作业4 Bézier 曲线](#作业4-Bézier-曲线)
      * [Basic](#Basic-1)
      * [Bonus](#Bonus-Antialiasing)
   * [作业5 光线追踪-光线与三角形相交](#作业5-光线追踪-光线与三角形相交)

# 参考作业实现

## 作业0 环境配置
- OS: Arch Linux x86_64
- kernel: latest
- editor: neovim
- compiler: clang/gcc
- lib: eigen

用虚拟机太麻烦了，自己装个eigen、opencv库就好了

Arch下eigen、opencv安装: ```sudo pacman -Sy eigen opencv```

正如老师第一节课中说的，还是推荐大家用ide

~~我用vim只是因为我有很多插件用习惯了, 顺手......~~

VIM如果使用clangd插件发现补全因为找不到头文件出错，可以尝试以下两种方法之一：

- 添加compile_flags.txt文件。告诉clangd编译参数，就像下面这样写入

```txt
-xc++
-I
/usr/include/opencv4/
-O3
-std=c++17
```

- 添加compile_commands.json文件。该文件可以在CMakeLists.txt中添加以下参数自动生成

```txt
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
```

其他VIM插件的介绍与该项目无关，可以参考我的[dotfiles](https://github.com/horel/dotfiles/tree/master/.config/nvim)

![](./doc/images/Hw0.png)

## 作业1 旋转与投影

![](./doc/images/Hw1.png)

## 作业2 Triangles and Z-buffering

### Basic

![](./doc/images/Hw2_basic.png)

### MSAA 4X

![](./doc/images/Hw2_MSAA_4X.png)

> 可以看到开启MSAA 4X后，锯齿感降低很多。同时增加以下部分，消除了交界处的黑边bug：
>
> - depth_buf_msaa2x2: depth buffer for MSAA
> - mix_pixel(): added for MSAA

## 作业3 Pipeline and Shading

### texture

![](doc/images/Hw3_texture.png)

<details>
    <summary>点击查看作业3其他材质图片（已折叠）</summary>

### normal

![](doc/images/Hw3_normal.png)

### phong

![](doc/images/Hw3_phong.png)

### bump

![](doc/images/Hw3_bump.png)

### displacement

![](doc/images/Hw3_displacement.png)

</details>

### Bonus1 Other models

> 特别注意：
> 提高题容易出现问题，具体请查看[CheckList](Hw3/README.md)

- bunny

![](doc/images/Hw3_bunny_normal.png)

<details>
    <summary>点击查看作业3其他Bonus图片（已折叠）</summary>

- Crate

![](doc/images/Hw3_crate_texture.png)

- cube

![](doc/images/Hw3_cube_texture.png)

- rock

![](doc/images/Hw3_rock_texture.png)

- Mobius[彩蛋]

![](doc/images/Hw3_Mobius.png)

> 梅比乌斯。贴图有很多张，框架里texture应该只能读一张，就只渲染了normal

### Bonus2 Bilinear

![](doc/images/Hw3_spot_texture_bilinear.png)

</details>

## 作业4 Bézier 曲线

### Basic

![](doc/images/Hw4_Bezier_Curve_4.png)

### Bonus Antialiasing

![](doc/images/Hw4_Bezier_Curve_Antialiasing.png)

## 作业5 光线追踪-光线与三角形相交

![](doc/images/Hw5.png)

## 作业6 光线追踪-加速结构
