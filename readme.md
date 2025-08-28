\# LingXiao-Crane-competition2025

2025年起重机赛“凌霄”作品，<b>hzq</b>个人部分。<br>

注1：由于设计为多MCU协作，提交代码偏重于个人编写或测试贡献较大的部分代码，因此部分代码为旧版本。如需要项目完整代码可以参考<br><b>韩kw</b>的提交（他应该提交了完整代码）<br>

注2：由于项目主要工作集中于连调阶段的各种抽象问题，因此“问题与解决方案”部分将涉及部分硬件层面的问题。<br><br>


文件架构：<br>

&nbsp;&nbsp;&nbsp;&nbsp;front：SM40BL+M2006<br>

&nbsp;&nbsp;&nbsp;&nbsp;general\_control：主控<br>

&nbsp;&nbsp;&nbsp;&nbsp;jixiebi：机械臂、起降机构、旋转机构<br>

&nbsp;&nbsp;&nbsp;&nbsp;motor：左右控制M2006<br>

&nbsp;&nbsp;&nbsp;&nbsp;wheel：前进后退控制<br>



个人贡献：<br>
1.motor文件夹中程序的M2006控制代码移植（基于战队群文件代码）与调试。<br>
2.front文件夹中程序的M2006控制代码（与1相同）、SM40BL控制代码的设计与调试。（二者main函数的整合由韩kw完成）<br>
3.jixiebi文件夹中程序的小米电机控制代码移植（基于小米电机手册与个人理解）与测试（由于后续步进改为小米电机，该文件夹与最终版不同，替换步进的小米电机控制代码由卞cc完成）。<br>
4.wheel文件夹中程序的stp\_23l数据接收代码的移植与修改（本人完成单接收部分，双接收部分的堆栈设计由卞cc完成）。<br>
5.general\_control文件夹中代码的测试与纠错（该代码由韩kw结合AI完成，本人仅协助完成工程调试）。<br>
