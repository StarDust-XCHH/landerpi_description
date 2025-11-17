# landerpi_description
这是一个基于Landerpi机器人的urdf描述文件的启动项修复，使之可以被简便的应用于常规电脑（而非官方的虚拟环境中）

(如果侵权了请联系我，我会及时删除，不过你司提供的例程代码bug真的多哈)


# 使用方法


## step0：移植

直接把本项目作为pkg移植到ros2的工作空间中

## step1：配置环境变量（根据想模拟的车型选择）或者写入bash配置

```bash
export MACHINE_TYPE=LanderPi_Mecanum
```
or

```bash
echo 'export MACHINE_TYPE=LanderPi_Mecanum' >> ~/.bashrc
source ~/.bashrc
```

## step2：编译

请于工作目录下执行`colcon build`进行编译

## step3：执行

```bash
ros2 launch landerpi_description display.launch.py
```

## step4：操作

稍作等待，直接使用官方的gui进行对关节和车轮的控制操作，以进行验证模型是否载入顺利

