


import QtQuick.Window
import QtQuick.Controls
import QtPositioning
import Viewer3D.Models3D

import Viewer3D

import QGroundControl
import QGroundControl.Controls
import QGroundControl.MultiVehicleManager
import QGroundControl.ScreenTools
import QGroundControl.Palette
import QGroundControl.FactSystem

import QGroundControl.Controllers // Mavlinktest

import QtCharts
import QtQuick 2.15
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window
import QtQuick3D 6.8
import QtQuick3D.Helpers

//import Qt3D.Core 2.15
//import Qt3D.Render 2.15
//import Qt3D.Extras 2.15
import QtQuick3D.Physics 6.8
import QtQuick3D.AssetUtils 6.8
Window {
    id: root
    title: "我的编队"
    width: 1600
    height: 900
    minimumWidth: 1200
    minimumHeight: 700
    visible: false
    color: "#1e1e2e"  // 深色主题背景

    // 样式常量
    property color primaryColor: "#5e81ac"  // 主色调蓝
    property color secondaryColor: "#a3be8c"  // 辅助色绿
    property color accentColor: "#ebcb8b"   // 强调色黄
    property color dangerColor: "#bf616a"   // 危险操作红
    property color textColor: "#eceff4"     // 文字颜色
    property color panelColor: "#2e3440"    // 面板颜色
    property color controlColor: "#3b4252"  // 控件颜色

    property color modelColor1: "#FBB72F"  // 模型颜色
    property color modelColor2: "#30677A"  // 模型颜色
    property color modelColor3: "#EA8C89"  // 模型颜色
    property color modelColor4: "#3be292"  // 模型颜色
    
    // 各组连接的模型数量
    property int group1Count: 0
    property int group2Count: 0
    property int group3Count: 0
    property int group4Count: 0
    
    // 更新各组数量
    function updateGroupCounts() {
        var c1 = 0, c2 = 0, c3 = 0, c4 = 0;
        for (var i = 0; i < plan_arr.length; i++) {
            if (plan_arr[i].is_connected) {
                if (plan_arr[i].group_id === 1) c1++;
                else if (plan_arr[i].group_id === 2) c2++;
                else if (plan_arr[i].group_id === 3) c3++;
                else if (plan_arr[i].group_id === 4) c4++;
            }
        }
        group1Count = c1;
        group2Count = c2;
        group3Count = c3;
        group4Count = c4;
    }

    // 您的属性和信号保持不变
    signal message()
    signal update_other_airplane(int param, int isset, int grp)


    Mavlinktest2 { id: test_mavlink }
    Swarmsend { id: swarm_send }

    // 主布局
    Rectangle {
        anchors.fill: parent
        color: "transparent"

        // 顶部控制栏
        Rectangle {
            id: topBar
            width: parent.width
            height: 80
            color: root.panelColor
            radius: 8

            RowLayout {
                anchors.fill: parent
                anchors.margins: 12
                spacing: 15

                // 飞行控制按钮组
                // RowLayout {
                //     spacing: 10
                //     CustomButton { text: "自动"; color: secondaryColor; onClicked: test_mavlink._sendcom(1,0,0,0,0) }
                //     CustomButton { text: "开始"; color: secondaryColor; onClicked: test_mavlink._sendcom(0,1,0,0,0) }
                //     CustomButton { text: "停止"; color: dangerColor; onClicked: test_mavlink._sendcom(0,0,1,0,0) }
                //     CustomButton { text: "选定主机"; color: accentColor }
                // }

                // 筹划管理
                RowLayout {
                    spacing: 10
                    CustomButton {
                        text: "筹划";
                        color: root.primaryColor;
                        onClicked: {
                            if (Number(input_plan.text) > 50) {
                                group_popu4.open()
                            } else {
                                plan_to_visible()
                            }
                        }
                    }

                    CustomTextField {
                        id: input_plan
                        text: "0"
                        width: 50
                        validator: IntValidator { bottom: 0; top: 50 }
                    }

                    Label {
                        text: "架"
                        color: root.textColor
                        font.pixelSize: 14
                    }
                }

               // RowLayout { // 第一组几个
                    RadiusButton {// 第一组
                        text: "";
                        color: modelColor1;// 选择第一组的颜色
                        enabled:false

                    }


                    Label {
                        text: "第一组 " + group1Count + " 架"
                        color: root.textColor
                        font.pixelSize: 14
                    }

                    RadiusButton {// 第二组
                        text: "";
                        color: root.modelColor2; //第二组颜色需改变
                        enabled:false

                    }


                    Label {
                        text: "第二组 " + group2Count + " 架"
                        color: root.textColor
                        font.pixelSize: 14
                    }

                    RadiusButton {// 第三组
                        text: "";
                        color: modelColor3; //第三组颜3色需改变
                        enabled:false

                    }


                    Label {
                        text: "第三组 " + group3Count + " 架"
                        color: root.textColor
                        font.pixelSize: 14
                    }

                    RadiusButton {// 第四组
                        id:modelbutton4
                        text: "";
                        color: modelColor4; //第四组颜色需改变
                        enabled:false
                       // visible: false
                    }


                    Label {
                        id:modellable4
                        text: "第四组 " + group4Count + " 架"
                        color: root.textColor
                        font.pixelSize: 14
                       // visible: false
                    }
               // }
                // 参数设置
            /*    RowLayout {
                    spacing: 10


                    Label {
                        text: "间距:"
                        color: textColor
                        font.pixelSize: 14
                    }

                    CustomTextField {
                        id: input1
                        text: "1"
                        width: 60
                        validator: IntValidator { bottom: 1; top: 100 }
                    }

                    Label {
                        text: "米"
                        color: root.textColor
                        font.pixelSize: 14
                    }

                    Label {
                        text: "高度:"
                        color: root.textColor
                        font.pixelSize: 14
                    }

                    CustomTextField {
                        id: input2
                        text: "0"
                        width: 60
                        validator: IntValidator { bottom: 0; top: 500 }
                    }

                    Label {
                        text: "米"
                        color: root.textColor
                        font.pixelSize: 14
                    }

                    CustomButton {
                        text: "设置高度"
                        width: 100
                        color: primaryColor
                    }
                }*/

                // // 飞行管理
                // RowLayout {
                //     spacing: 10
                //     CustomButton {
                //         text: "暂停";
                //         color: root.accentColor;
                //         onClicked: test_mavlink._sendcom2(0,0,0,separate_main,0)
                //     }
                //     CustomButton {
                //         text: "继续";
                //         color: secondaryColor;
                //         onClicked: test_mavlink._sendcom(0,0,0,0,separate_main)
                //     }
                // }


            }
        }

        // 主内容区域 - 3D视图
        Rectangle {
            id: viewContainer
            anchors {
                top: topBar.bottom
                bottom: bottomBar.top
                // left: parent.left
                // right: controlPanel.left
                left: controlPanel.right
                right: parent.right
                margins: 12
            }

                // 超出部分裁剪（可选）
                clip: true

            color: "#252538"
            radius: 8
            border.color: "#4c566a"
            border.width: 1

            View3D {
                id: control
                anchors.fill:parent
                //背景
                environment: SceneEnvironment {
                   clearColor: "skyblue"
                   backgroundMode: SceneEnvironment.Color
                }
                Component.onCompleted: {
                        console.log("View3D实际尺寸：宽=", width, "高=", height);
                    }

                onWidthChanged: {
                            if (width > 0) {
                                console.log("View3D宽度更新：", width);
                                // 此处可添加尺寸变化后的业务逻辑

                                mymove(1,1,sphere_node)
                                get_pos(sphere_node);

                                mymove(2,1,sphere_node2)
                                get_pos(sphere_node2);
                                mymove(3,1,sphere_node3)
                                get_pos(sphere_node3);
                                mymove(4,1,sphere_node4)
                                get_pos(sphere_node4);
                                mymove(5,1,sphere_node5)
                                get_pos(sphere_node5);
                                mymove(6,1,sphere_node6)
                                get_pos(sphere_node6);
                                mymove(7,1,sphere_node7)
                                get_pos(sphere_node7);
                                mymove(8,1,sphere_node8)
                                get_pos(sphere_node8);
                                mymove(9,1,sphere_node9)
                                get_pos(sphere_node9);
                                mymove(10,1,sphere_node10)
                                get_pos(sphere_node10);
                                mymove(11,1,sphere_node11)
                                get_pos(sphere_node11);
                                mymove(12,1,sphere_node12)
                                get_pos(sphere_node12);
                                mymove(13,1,sphere_node13)
                                get_pos(sphere_node13);
                                mymove(14,1,sphere_node14)
                                get_pos(sphere_node14);
                                mymove(15,1,sphere_node15)
                                get_pos(sphere_node15);
                                mymove(16,1,sphere_node16)
                                get_pos(sphere_node16);
                                mymove(17,1,sphere_node17)
                                get_pos(sphere_node17);
                                mymove(18,1,sphere_node18)
                                get_pos(sphere_node18);
                                mymove(19,1,sphere_node19)
                                get_pos(sphere_node19);
                                mymove(20,1,sphere_node20)
                                get_pos(sphere_node20);
                                mymove(21,1,sphere_node21)
                                get_pos(sphere_node21);
                                mymove(22,1,sphere_node22)
                                get_pos(sphere_node22);
                                mymove(23,1,sphere_node23)
                                get_pos(sphere_node23);
                                mymove(24,1,sphere_node24)
                                get_pos(sphere_node24);
                                mymove(25,1,sphere_node25)
                                get_pos(sphere_node25);
                                mymove(26,1,sphere_node26)
                                get_pos(sphere_node26);
                                mymove(27,1,sphere_node27)
                                get_pos(sphere_node27);
                                mymove(28,1,sphere_node28)
                                get_pos(sphere_node28);
                                mymove(1,2,sphere_node29)
                                get_pos(sphere_node29);
                                mymove(2,2,sphere_node30)
                                get_pos(sphere_node30);
                                mymove(3,2,sphere_node31)
                                get_pos(sphere_node2);
                                mymove(4,2,sphere_node32)
                                get_pos(sphere_node32);
                                mymove(5,2,sphere_node33)
                                get_pos(sphere_node33);
                                mymove(6,2,sphere_node34)
                                get_pos(sphere_node34);
                                mymove(7,2,sphere_node35)
                                get_pos(sphere_node35);
                                mymove(8,2,sphere_node36)
                                get_pos(sphere_node36);
                                mymove(9,2,sphere_node37)
                                get_pos(sphere_node37);
                                mymove(10,2,sphere_node38)
                                get_pos(sphere_node38);
                                mymove(11,2,sphere_node39)
                                get_pos(sphere_node39);
                                mymove(12,2,sphere_node40)
                                get_pos(sphere_node40);
                                mymove(13,2,sphere_node41)
                                get_pos(sphere_node41);
                                mymove(14,2,sphere_node42)
                                get_pos(sphere_node42);
                                mymove(15,2,sphere_node43)
                                get_pos(sphere_node43);
                                mymove(16,2,sphere_node44)
                                get_pos(sphere_node44);
                                mymove(17,2,sphere_node45)
                                get_pos(sphere_node45);
                                mymove(18,2,sphere_node46)
                                get_pos(sphere_node46);
                                mymove(19,2,sphere_node47)
                                get_pos(sphere_node47);
                                mymove(20,2,sphere_node48)
                                get_pos(sphere_node48);
                                mymove(21,2,sphere_node49)
                                get_pos(sphere_node49);
                                mymove(22,2,sphere_node50)
                                get_pos(sphere_node50);

                            }
                        }
                GridView{
                   id:mygrid
                   anchors.fill: parent
                   anchors.margins: 0

                   clip: true　　// 设置clip属性为true，来激活裁剪功能

                   model:1500
                   delegate: numberDelegate
                   cellHeight: 40
                   cellWidth: 40
                }
                           Component{
                               id:numberDelegate
                               Rectangle{
                                   id:rct
                                   width: 40;
                                   height: 40;
                                   color: "Transparent"
                                   Text {
                                       id:txt
                                       anchors.centerIn: parent
                                       font.pixelSize: 15
                                       text: "+"
                                        color: "#dae0eb"

                                   }
                                   function setText(newtext) {
                                       txt.text=newtext
                                   }
                               }
                           }

                       //观察相机
                       //View3D的mapTo/mapFrom坐标转换函数需要先设置camera属性
                       camera: perspective_camera
                       PerspectiveCamera {
                           id: perspective_camera
                         //  z:   control.height * 3.5   //1800左右
                           z:1800
                          // aspectRatio: view3D.width / view3D.height
                       }
                       //光照
                       DirectionalLight {
                           eulerRotation.z: 0
                       }

                       Canvas{
                           id:canv
                           anchors.fill: parent
                           visible: false
                           onPaint: { //    |的上半部分
                               var vtx = getContext("2d")
                               vtx.strokeStyle = "black"
                               vtx.linewidth = 2
                               var startX = control.width / 2
                               var startY = 0

                               var endX = control.width / 2
                               var endY =(control.height ) / 2

                               vtx.beginPath()
                               vtx.moveTo(startX,startY)
                               vtx.lineTo(endX,endY)
                               vtx.stroke()
                           }
                       }

                       Canvas{  //    |的下半部分
                           id:canv2
                           anchors.fill: parent
                           visible: false
                           onPaint: {
                               var vtx = getContext("2d")
                               vtx.strokeStyle = "black"
                               vtx.linewidth = 2
                               var startX = control.width / 2
                               var startY = (control.height ) / 2

                               var endX = control.width / 2
                               var endY = control.height

                               vtx.beginPath()
                               vtx.moveTo(startX,startY)
                               vtx.lineTo(endX,endY)
                               vtx.stroke()
                           }
                       }

                       Canvas{
                           id:canv3  //    ——的左半部分
                           anchors.fill: parent
                           visible: false
                           onPaint: {
                               var vtx = getContext("2d")
                               vtx.strokeStyle = "black"
                               vtx.linewidth = 2
                               var startX = 0
                               var startY = (control.height ) / 2

                               var endX = control.width / 2
                               var endY = (control.height ) / 2

                               vtx.beginPath()
                               vtx.moveTo(startX,startY)
                               vtx.lineTo(endX,endY)
                               vtx.stroke()
                           }
                       }
                       Canvas{
                           id:canv4  //    ——的右半部分
                           anchors.fill: parent
                           visible: false
                           onPaint: {
                               var vtx = getContext("2d")
                               vtx.strokeStyle = "black"
                               vtx.linewidth = 2
                               var startX = control.width / 2
                               var startY = (control.height ) / 2

                               var endX = control.width
                               var endY = (control.height ) / 2

                               vtx.beginPath()
                               vtx.moveTo(startX,startY)
                               vtx.lineTo(endX,endY)
                               vtx.stroke()
                           }
                       }

                           //children:[]
                           // RuntimeLoader {
                           //     id: myVehicleInstance // 给车辆实例一个唯一ID，方便后续引用
                           //     source: "myvehicle.glb"
                           //     eulerRotation.x: 90 // 模型姿态调整
                           //     position: Qt.vector3d(0, 0, 0) // 初始位置
                           //      scale: Qt.vector3d(6, 6, 6)


                           //              // 添加ObjectPicker组件
                           //              ObjectPicker {
                           //                  id: modelPicker
                           //                  dragEnabled: true
                           //                  hoverEnabled: true
                           //                  priority: 1

                           //                  onPressed: (pick) => {
                           //                      console.log("Pressed at:", pick.worldIntersection);
                           //                      // 处理按下事件
                           //                  }
                           //                  onReleased: (pick) => {
                           //                      console.log("Released at:", pick.worldIntersection);
                           //                      // 处理释放事件
                           //                  }
                           //                  onMoved: (pick) => {
                           //                      // 处理移动事件
                           //                  }
                           //              }

                           //     // 通过status属性检查加载状态
                           //     onStatusChanged: {
                           //         if (status === RuntimeLoader.Success) {
                           //             console.log("模型加载成功")
                           //         } else if (status === RuntimeLoader.Error) {
                           //             console.log("加载失败:", errorString)
                           //         }
                           //     }
                           // }

                       Model {
                           Text {
                               id: name
                               text: "  " + sphere_node.objectName + "_" + sphere_node.group_id
                               font.pixelSize: 62
                               color: sphere_node.set_main ? "red":"black"
                           }
                           id: sphere_node
                           objectName: "1"
                           source: "#Sphere"
                           pickable: true
                           x: 0
                           y: 1
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           z: 5
                           visible: true
                           function loadModel() {
                               visible = true;
                           }

                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool is_main: false
                           property bool set_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                          // property color darkcolor: Qt.darker("#646566",1.2)
                           materials: DefaultMaterial {
                               opacity: sphere_node.select_color
                               diffuseColor: sphere_node.is_connected ? (sphere_node.is_main ? "red" : (sphere_node.group_id === 1 ? modelColor1 : (sphere_node.group_id === 2 ? modelColor2 : (sphere_node.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"

                           }
                          // materials: edgeColor:Qt.rgba(1.0,0.0,0.0,1.0)
                         //  materials:
                           Component.onCompleted: {
                               // mymove(1,1,sphere_node) // 在这去更新 x y z
                               // get_pos(sphere_node);
                             //  console.log(_activeVehicle.Vehicle_count) // 好像没用了

                             //  _sysid_list.push(1)     测
                             // idpos_map[1] = [2,2,0]   试用

                             //  screen_pos_to_world_pos(1,1,sphere_node)
                               hasset_map[1]=0
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // bug：所有模型都连此信号时，当信号发过来时，所有已显示的模型名字更新 且一样  应该已解决，待测试
                                   if (n === 1) {
                                       if (if_main_node(sphere_node.objectName)) {sphere_node.is_main = true;reset_main_name(sphere_node.objectName,sysid)} // 如果已设置主机,连上时需更新曾设的主机名,因为连上前和连上后的id并非一致
                                       sphere_node.objectName = sysid
                                       _sysid_list.push(sysid)  // 所有id 集合
                                       sphere_node.is_connected = true
                                       sphere_node.pickable = true
                                       idpos_map[sysid] = [sphere_node.model_x,sphere_node.model_y,sphere_node.model_z]
                                       modelmp[sysid] = sphere_node
                                       swarm_send.store_airplane_group(sysid, sphere_node.group_id,false)

                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node.set_main,sphere_node.group_id)
                                       else
                                           update_other_airplane(sphere_node.set_main,sphere_node.set_main,sphere_node.group_id)

                                       console.log("pos1",sphere_node.model_x,sphere_node.model_y,sphere_node.model_z)
                                     //  update_other_airplane(sphere_node.set_main)
                                    //   if (sphere_node.set_main)set_main_behavior(sphere_node) // 时机不对,要全连接了再排他

                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node.objectName)) {
                                       sphere_node.objectName = "1"
                                       sphere_node.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name2
                               text: "  " + sphere_node2.objectName + "_" + sphere_node2.group_id
                               font.pixelSize: 62
                               color: sphere_node2.set_main ? "red":"black"
                           }
                           id: sphere_node2
                           objectName: "2"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           y: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int group_id: 1
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property bool is_main: false
                           property bool set_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                           //    diffuseColor:"cyan"
                               opacity: sphere_node2.select_color
                               diffuseColor: sphere_node2.is_connected ? (sphere_node2.is_main ? "red" : (sphere_node2.group_id === 1 ? modelColor1 : (sphere_node2.group_id === 2 ? modelColor2 : (sphere_node2.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }
                           Component.onCompleted: {
                               // mymove(2,1,sphere_node2)
                               // get_pos(sphere_node2);
                             //  idpos_map[2] = [sphere_node2.model_x,sphere_node2.model_y,sphere_node2.model_z] // 测试用
                             //  _sysid_list.push(2)     测
                             // idpos_map[2] = [4,4,5]   试用
                            //   screen_pos_to_world_pos(2,1,sphere_node2)
                           }

                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 2) {
                                       if (if_main_node(sphere_node2.objectName)) {sphere_node2.is_main = true;reset_main_name(sphere_node2.objectName,sysid)}
                                       sphere_node2.objectName = sysid
                                       _sysid_list.push(sysid)
                                      // sphere_node2.pickable = true
                                       sphere_node2.is_connected = true
                                       idpos_map[sysid] = [sphere_node2.model_x,sphere_node2.model_y,sphere_node2.model_z]
                                       modelmp[sysid] = sphere_node2
                                       swarm_send.store_airplane_group(sysid, sphere_node2.group_id,false)

                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node2.set_main,sphere_node2.group_id)
                                       else
                                           update_other_airplane(sphere_node2.set_main,sphere_node2.set_main,sphere_node2.group_id)

                                       console.log("pos2",sphere_node2.model_x,sphere_node2.model_y,sphere_node2.model_z)
                                     //  update_other_airplane(sphere_node2.set_main)
                                     //  if (sphere_node2.set_main)set_main_behavior(sphere_node2)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node2.objectName)) {
                                       sphere_node2.objectName = "2"
                                       sphere_node2.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node2.group_id = 1
                                   }
                               }
                           }

                       }
                       Model {
                           Text {
                               id: name3
                               text: "  " + sphere_node3.objectName + "_" + sphere_node3.group_id
                               font.pixelSize: 62
                               color: sphere_node3.set_main ? "red":"black"
                           }
                           id: sphere_node3
                           objectName: "3"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int group_id: 1
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property bool is_main: false
                           property bool set_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node3.select_color
                              // diffuseColor:sphere_node3.is_main ? "red" : "cyan"
                               diffuseColor: sphere_node3.is_connected ? (sphere_node3.is_main ? "red" : (sphere_node3.group_id === 1 ? modelColor1 : (sphere_node3.group_id === 2 ? modelColor2 : (sphere_node3.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(3,1,sphere_node3)
                               // get_pos(sphere_node3);

                             //  screen_pos_to_world_pos(3,1,sphere_node3)
                            //   _sysid_list.push(3)   测
                            //  idpos_map[3] = [4,4,0] 试用
                             //  idpos_map[3] = [sphere_node3.model_x,sphere_node3.model_y,sphere_node3.model_z]
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 3) {
                                       if (if_main_node(sphere_node3.objectName)) {sphere_node3.is_main = true;reset_main_name(sphere_node3.objectName,sysid)}
                                       sphere_node3.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node3.pickable = true
                                       sphere_node3.is_connected = true
                                       idpos_map[sysid] = [sphere_node3.model_x,sphere_node3.model_y,sphere_node3.model_z]
                                       modelmp[sysid] = sphere_node3
                                       swarm_send.store_airplane_group(sysid, sphere_node3.group_id,false)

                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node3.set_main,sphere_node3.group_id)
                                       else
                                           update_other_airplane(sphere_node3.set_main,sphere_node3.set_main,sphere_node3.group_id)
                                      console.log("pos3",sphere_node3.model_x,sphere_node3.model_y,sphere_node3.model_z)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node3.objectName)) {
                                       sphere_node3.objectName = "3"
                                       sphere_node3.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node3.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name4
                               text: "  " + sphere_node4.objectName + "_" + sphere_node4.group_id
                               font.pixelSize: 62
                               color: sphere_node4.set_main ? "red":"black"
                           }
                           id: sphere_node4
                           objectName: "4"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool is_main: false
                           property bool set_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node4.select_color
                              // diffuseColor: sphere_node4.is_main ? "red" : "cyan"
                               diffuseColor: sphere_node4.is_connected ? (sphere_node4.is_main ? "red" : (sphere_node4.group_id === 1 ? modelColor1 : (sphere_node4.group_id === 2 ? modelColor2 : (sphere_node4.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(4,1,sphere_node4)
                               // get_pos(sphere_node4);
                             //  screen_pos_to_world_pos(4,1,sphere_node4)
                             //  idpos_map[4] = [sphere_node4.model_x,sphere_node4.model_y,sphere_node4.model_z]
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 4) {
                                       if (if_main_node(sphere_node4.objectName)) {sphere_node4.is_main = true;reset_main_name(sphere_node4.objectName,sysid)}
                                       sphere_node4.objectName = sysid
                                       _sysid_list.push(sysid)
                                       sphere_node4.is_connected = true
                                      // sphere_node4.pickable = true
                                       idpos_map[sysid] = [sphere_node4.model_x,sphere_node4.model_y,sphere_node4.model_z]
                                       modelmp[sysid] = sphere_node4
                                       swarm_send.store_airplane_group(sysid, sphere_node4.group_id,false)

                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node4.set_main,sphere_node4.group_id)
                                       else
                                           update_other_airplane(sphere_node4.set_main,sphere_node4.set_main,sphere_node4.group_id)
                                     console.log("pos4",sphere_node4.model_x,sphere_node4.model_y,sphere_node4.model_z)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node4.objectName)) {
                                       sphere_node4.objectName = "4"
                                       sphere_node4.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node4.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name5
                               text: "  " + sphere_node5.objectName + "_" + sphere_node5.group_id
                               font.pixelSize: 62
                               color: sphere_node5.set_main ? "red":"black"
                           }
                           id: sphere_node5
                           objectName: "5"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool is_main: false
                           property bool set_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node5.select_color
                               diffuseColor: sphere_node5.is_connected ? (sphere_node5.is_main ? "red" : (sphere_node5.group_id === 1 ? modelColor1 : (sphere_node5.group_id === 2 ? modelColor2 : (sphere_node5.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(5,1,sphere_node5)
                               // get_pos(sphere_node5);
                             //  screen_pos_to_world_pos(5,1,sphere_node5)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 5) {
                                       if (if_main_node(sphere_node5.objectName)) {sphere_node5.is_main = true;reset_main_name(sphere_node5.objectName,sysid)}
                                       sphere_node5.objectName = sysid
                                       _sysid_list.push(sysid)
                                       sphere_node5.is_connected = true
                                      // sphere_node5.pickable = true
                                       idpos_map[sysid] = [sphere_node5.model_x,sphere_node5.model_y,sphere_node5.model_z]
                                       modelmp[sysid] = sphere_node5
                                       swarm_send.store_airplane_group(sysid, sphere_node5.group_id,false)

                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node5.set_main,sphere_node5.group_id)
                                       else
                                           update_other_airplane(sphere_node5.set_main,sphere_node5.set_main,sphere_node5.group_id)
                                    console.log("pos5",sphere_node5.model_x,sphere_node5.model_y,sphere_node5.model_z)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node5.objectName)) {
                                       sphere_node5.objectName = "5"
                                       sphere_node5.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node5.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name6
                               text: "  " + sphere_node6.objectName + "_" + sphere_node6.group_id
                               font.pixelSize: 62
                               color: sphere_node6.set_main ? "red":"black"
                           }
                           id: sphere_node6
                           objectName: "6"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool is_main: false
                           property bool set_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node6.select_color
                               diffuseColor: sphere_node6.is_connected ? (sphere_node6.is_main ? "red" : (sphere_node6.group_id === 1 ? modelColor1 : (sphere_node6.group_id === 2 ? modelColor2 : (sphere_node6.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(6,1,sphere_node6)
                               // get_pos(sphere_node6);
                            //   screen_pos_to_world_pos(6,1,sphere_node6)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 6) {
                                       if (if_main_node(sphere_node6.objectName)) {sphere_node6.is_main = true;reset_main_name(sphere_node6.objectName,sysid)}
                                       sphere_node6.objectName = sysid
                                       _sysid_list.push(sysid)
                                      // sphere_node6.pickable = true
                                       sphere_node6.is_connected = true
                                       idpos_map[sysid] = [sphere_node6.model_x,sphere_node6.model_y,sphere_node6.model_z]
                                       modelmp[sysid] = sphere_node6
                                       swarm_send.store_airplane_group(sysid, sphere_node6.group_id,false)

                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node6.set_main,sphere_node6.group_id)
                                       else
                                           update_other_airplane(sphere_node6.set_main,sphere_node6.set_main,sphere_node6.group_id)
                                     console.log("pos6",sphere_node6.model_x,sphere_node6.model_y,sphere_node6.model_z)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node6.objectName)) {
                                       sphere_node6.objectName = "6"
                                       sphere_node6.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node6.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name7
                               text: "  " + sphere_node7.objectName + "_" + sphere_node7.group_id
                               font.pixelSize: 62
                               color: sphere_node7.set_main ? "red":"black"
                           }
                           id: sphere_node7
                           objectName: "7"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool is_main: false
                           property bool set_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node7.select_color
                               diffuseColor: sphere_node7.is_connected ? (sphere_node7.is_main ? "red" : (sphere_node7.group_id === 1 ? modelColor1 : (sphere_node7.group_id === 2 ? modelColor2 : (sphere_node7.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(7,1,sphere_node7)
                               // get_pos(sphere_node7);
                             //  screen_pos_to_world_pos(7,1,sphere_node7)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 7) {
                                       if (if_main_node(sphere_node7.objectName)) {sphere_node7.is_main = true;reset_main_name(sphere_node7.objectName,sysid)}
                                       sphere_node7.objectName = sysid
                                       _sysid_list.push(sysid)
                                      // sphere_node7.pickable = true
                                       sphere_node7.is_connected = true
                                       idpos_map[sysid] = [sphere_node7.model_x,sphere_node7.model_y,sphere_node7.model_z]
                                       modelmp[sysid] = sphere_node7
                                       swarm_send.store_airplane_group(sysid, sphere_node7.group_id,false)

                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node7.set_main,sphere_node7.group_id)
                                       else
                                           update_other_airplane(sphere_node7.set_main,sphere_node7.set_main,sphere_node7.group_id)
                                     console.log("pos7",sphere_node7.model_x,sphere_node7.model_y,sphere_node7.model_z)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node7.objectName)) {
                                       sphere_node7.objectName = "7"
                                       sphere_node7.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node7.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name8
                               text: "  " + sphere_node8.objectName + "_" + sphere_node8.group_id
                               font.pixelSize: 62
                               color: sphere_node8.set_main ? "red":"black"
                           }
                           id: sphere_node8
                           objectName: "8"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool is_main: false
                           property bool set_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node8.select_color
                               diffuseColor: sphere_node8.is_connected ? (sphere_node8.is_main ? "red" : (sphere_node8.group_id === 1 ? modelColor1 : (sphere_node8.group_id === 2 ? modelColor2 : (sphere_node8.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(8,1,sphere_node8)
                               // get_pos(sphere_node8);
                             //  screen_pos_to_world_pos(8,1,sphere_node8)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 8) {
                                       if (if_main_node(sphere_node8.objectName)) {sphere_node8.is_main = true;reset_main_name(sphere_node8.objectName,sysid)}
                                       sphere_node8.objectName = sysid
                                       _sysid_list.push(sysid)
                                       sphere_node8.is_connected = true
                                       idpos_map[sysid] = [sphere_node8.model_x,sphere_node8.model_y,sphere_node8.model_z]
                                       modelmp[sysid] = sphere_node8
                                       swarm_send.store_airplane_group(sysid, sphere_node8.group_id,false)

                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node8.set_main,sphere_node8.group_id)
                                       else
                                           update_other_airplane(sphere_node8.set_main,sphere_node8.set_main,sphere_node8.group_id)
                                     console.log("pos8",sphere_node8.model_x,sphere_node8.model_y,sphere_node8.model_z)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node8.objectName)) {
                                       sphere_node8.objectName = "8"
                                       sphere_node8.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node8.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name9
                               text: "  " + sphere_node9.objectName + "_" + sphere_node9.group_id
                               font.pixelSize: 62
                               color: sphere_node9.set_main ? "red":"black"
                           }
                           id: sphere_node9
                           objectName: "9"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node9.select_color
                               diffuseColor: sphere_node9.is_connected ? (sphere_node9.is_main ? "red" : (sphere_node9.group_id === 1 ? modelColor1 : (sphere_node9.group_id === 2 ? modelColor2 : (sphere_node9.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(9,1,sphere_node9)
                               // get_pos(sphere_node9);
                            //   screen_pos_to_world_pos(9,1,sphere_node9)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 9) {
                                       if (if_main_node(sphere_node9.objectName)) {sphere_node9.is_main = true;reset_main_name(sphere_node9.objectName,sysid)}
                                       sphere_node9.objectName = sysid
                                       _sysid_list.push(sysid)
                                       sphere_node9.is_connected = true
                                       idpos_map[sysid] = [sphere_node9.model_x,sphere_node9.model_y,sphere_node9.model_z]
                                       modelmp[sysid] = sphere_node9
                                       swarm_send.store_airplane_group(sysid, sphere_node9.group_id,false)

                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node9.set_main,sphere_node9.group_id)
                                       else
                                           update_other_airplane(sphere_node9.set_main,sphere_node9.set_main,sphere_node9.group_id)
                                     console.log("pos9",sphere_node9.model_x,sphere_node9.model_y,sphere_node9.model_z)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node9.objectName)) {
                                       sphere_node9.objectName = "9"
                                       sphere_node9.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node9.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name10
                               text: "  " + sphere_node10.objectName + "_" + sphere_node10.group_id
                               font.pixelSize: 62
                               color: sphere_node10.set_main ? "red":"black"
                           }
                           id: sphere_node10
                           objectName: "10"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node10.select_color
                               diffuseColor: sphere_node10.is_connected ? (sphere_node10.is_main ? "red" : (sphere_node10.group_id === 1 ? modelColor1 : (sphere_node10.group_id === 2 ? modelColor2 : (sphere_node10.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(10,1,sphere_node10)
                               // get_pos(sphere_node10);
                             //  screen_pos_to_world_pos(10,1,sphere_node10)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 10) {
                                       if (if_main_node(sphere_node10.objectName)) {sphere_node10.is_main = true;reset_main_name(sphere_node10.objectName,sysid)}
                                       sphere_node10.objectName = sysid
                                       _sysid_list.push(sysid)
                                       sphere_node10.is_connected = true
                                       idpos_map[sysid] = [sphere_node10.model_x,sphere_node10.model_y,sphere_node10.model_z]
                                       modelmp[sysid] = sphere_node10
                                       swarm_send.store_airplane_group(sysid, sphere_node10.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node10.set_main,sphere_node10.group_id)
                                       else
                                           update_other_airplane(sphere_node10.set_main,sphere_node10.set_main,sphere_node10.group_id)
                                       console.log("pos10",sphere_node10.model_x,sphere_node10.model_y,sphere_node10.model_z)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node10.objectName)) {
                                       sphere_node10.objectName = "10"
                                       sphere_node10.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node10.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name11
                               text: "  " + sphere_node11.objectName + "_" + sphere_node11.group_id
                               font.pixelSize: 62
                               color: sphere_node11.set_main ? "red":"black"
                           }
                           id: sphere_node11
                           objectName: "11"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node11.select_color
                               diffuseColor: sphere_node11.is_connected ? (sphere_node11.is_main ? "red" : (sphere_node11.group_id === 1 ? modelColor1 : (sphere_node11.group_id === 2 ? modelColor2 : (sphere_node11.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(11,1,sphere_node11)
                               // get_pos(sphere_node11);
                             //  screen_pos_to_world_pos(11,1,sphere_node11)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 11) {
                                       if (if_main_node(sphere_node11.objectName)) {sphere_node11.is_main = true;reset_main_name(sphere_node11.objectName,sysid)}
                                       sphere_node11.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node11.is_connected = true
                                       idpos_map[sysid] = [sphere_node11.model_x,sphere_node11.model_y,sphere_node11.model_z]
                                       modelmp[sysid] = sphere_node11
                                       swarm_send.store_airplane_group(sysid, sphere_node11.group_id,false)

                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node11.set_main,sphere_node11.group_id)
                                       else
                                           update_other_airplane(sphere_node11.set_main,sphere_node11.set_main,sphere_node11.group_id)
                                       console.log("pos11",sphere_node11.model_x,sphere_node11.model_y,sphere_node11.model_z)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node11.objectName)) {
                                       sphere_node11.objectName = "11"
                                       sphere_node11.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node11.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name12
                               text: "  " + sphere_node12.objectName + "_" + sphere_node12.group_id
                               font.pixelSize: 62
                               color: sphere_node12.set_main ? "red":"black"
                           }
                           id: sphere_node12
                           objectName: "12"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node12.select_color
                               diffuseColor: sphere_node12.is_connected ? (sphere_node12.is_main ? "red" : (sphere_node12.group_id === 1 ? modelColor1 : (sphere_node12.group_id === 2 ? modelColor2 : (sphere_node12.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(12,1,sphere_node12)
                               // get_pos(sphere_node12);
                             //  screen_pos_to_world_pos(12,1,sphere_node12)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 12) {
                                       if (if_main_node(sphere_node12.objectName)) {sphere_node12.is_main = true;reset_main_name(sphere_node12.objectName,sysid)}
                                       sphere_node12.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node12.is_connected = true
                                       idpos_map[sysid] = [sphere_node12.model_x,sphere_node12.model_y,sphere_node12.model_z]
                                       modelmp[sysid] = sphere_node12
                                       swarm_send.store_airplane_group(sysid, sphere_node12.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node12.set_main,sphere_node12.group_id)
                                       else
                                           update_other_airplane(sphere_node12.set_main,sphere_node12.set_main,sphere_node12.group_id)

                                       console.log("pos12",sphere_node12.model_x,sphere_node12.model_y,sphere_node12.model_z)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node12.objectName)) {
                                       sphere_node12.objectName = "12"
                                       sphere_node12.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node12.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name13
                               text: "  " + sphere_node13.objectName + "_" + sphere_node13.group_id
                               font.pixelSize: 62
                               color: sphere_node13.set_main ? "red":"black"
                           }
                           id: sphere_node13
                           objectName: "13"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node13.select_color
                               diffuseColor: sphere_node13.is_connected ? (sphere_node13.is_main ? "red" : (sphere_node13.group_id === 1 ? modelColor1 : (sphere_node13.group_id === 2 ? modelColor2 : (sphere_node13.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(13,1,sphere_node13)
                               // get_pos(sphere_node13);
                             //  screen_pos_to_world_pos(13,1,sphere_node13)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 13) {
                                       if (if_main_node(sphere_node13.objectName)) {sphere_node13.is_main = true;reset_main_name(sphere_node13.objectName,sysid)}
                                       sphere_node13.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node13.is_connected = true
                                       idpos_map[sysid] = [sphere_node13.model_x,sphere_node13.model_y,sphere_node13.model_z]
                                       modelmp[sysid] = sphere_node13
                                       swarm_send.store_airplane_group(sysid, sphere_node13.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node13.set_main,sphere_node13.group_id)
                                       else
                                           update_other_airplane(sphere_node13.set_main,sphere_node13.set_main,sphere_node13.group_id)
                                       console.log("pos13",sphere_node13.model_x,sphere_node13.model_y,sphere_node13.model_z)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node13.objectName)) {
                                       sphere_node13.objectName = "13"
                                       sphere_node13.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node13.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name14
                               text: "  " + sphere_node14.objectName + "_" + sphere_node14.group_id
                               font.pixelSize: 62
                               color: sphere_node14.set_main ? "red":"black"
                           }
                           id: sphere_node14
                           objectName: "14"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node14.select_color
                               diffuseColor: sphere_node14.is_connected ? (sphere_node14.is_main ? "red" : (sphere_node14.group_id === 1 ? modelColor1 : (sphere_node14.group_id === 2 ? modelColor2 : (sphere_node14.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(14,1,sphere_node14)
                               // get_pos(sphere_node14);
                            //   screen_pos_to_world_pos(14,1,sphere_node14)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 14) {
                                       if (if_main_node(sphere_node14.objectName)) {sphere_node14.is_main = true;reset_main_name(sphere_node14.objectName,sysid)}
                                       sphere_node14.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node14.is_connected = true
                                       idpos_map[sysid] = [sphere_node14.model_x,sphere_node14.model_y,sphere_node14.model_z]
                                       modelmp[sysid] = sphere_node14
                                       swarm_send.store_airplane_group(sysid, sphere_node14.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node14.set_main,sphere_node14.group_id)
                                       else
                                           update_other_airplane(sphere_node14.set_main,sphere_node14.set_main,sphere_node14.group_id)
                                       console.log("pos14",sphere_node14.model_x,sphere_node14.model_y,sphere_node14.model_z)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node14.objectName)) {
                                       sphere_node14.objectName = "14"
                                       sphere_node14.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node14.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name15
                               text: "  " + sphere_node15.objectName + "_" + sphere_node15.group_id
                               font.pixelSize: 62
                               color: sphere_node15.set_main ? "red":"black"
                           }
                           id: sphere_node15
                           objectName: "15"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node15.select_color
                               diffuseColor: sphere_node15.is_connected ? (sphere_node15.is_main ? "red" : (sphere_node15.group_id === 1 ? modelColor1 : (sphere_node15.group_id === 2 ? modelColor2 : (sphere_node15.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(15,1,sphere_node15)
                               // get_pos(sphere_node15);
                             //  screen_pos_to_world_pos(15,1,sphere_node15)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 15) {
                                       if (if_main_node(sphere_node15.objectName)) {sphere_node15.is_main = true;reset_main_name(sphere_node15.objectName,sysid)}
                                       sphere_node15.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node15.is_connected = true
                                       idpos_map[sysid] = [sphere_node15.model_x,sphere_node15.model_y,sphere_node15.model_z]
                                       modelmp[sysid] = sphere_node15
                                       swarm_send.store_airplane_group(sysid, sphere_node15.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node15.set_main,sphere_node15.group_id)
                                       else
                                           update_other_airplane(sphere_node15.set_main,sphere_node15.set_main,sphere_node15.group_id)
                                       console.log("pos15",sphere_node15.model_x,sphere_node15.model_y,sphere_node15.model_z)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node15.objectName)) {
                                       sphere_node15.objectName = "15"
                                       sphere_node15.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node15.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name16
                               text: "  " + sphere_node16.objectName + "_" + sphere_node16.group_id
                               font.pixelSize: 62
                               color: sphere_node16.set_main ? "red":"black"
                           }
                           id: sphere_node16
                           objectName: "16"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node16.select_color
                               diffuseColor: sphere_node16.is_connected ? (sphere_node16.is_main ? "red" : (sphere_node16.group_id === 1 ? modelColor1 : (sphere_node16.group_id === 2 ? modelColor2 : (sphere_node16.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(16,1,sphere_node16)
                               // get_pos(sphere_node16);
                             //  screen_pos_to_world_pos(16,1,sphere_node16)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 16) {
                                       if (if_main_node(sphere_node16.objectName)) {sphere_node16.is_main = true;reset_main_name(sphere_node16.objectName,sysid)}
                                       sphere_node16.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node16.is_connected = true
                                       idpos_map[sysid] = [sphere_node16.model_x,sphere_node16.model_y,sphere_node16.model_z]
                                       modelmp[sysid] = sphere_node16
                                       swarm_send.store_airplane_group(sysid, sphere_node16.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node16.set_main,sphere_node16.group_id)
                                       else
                                           update_other_airplane(sphere_node16.set_main,sphere_node16.set_main,sphere_node16.group_id)
                                       console.log("pos16",sphere_node16.model_x,sphere_node16.model_y,sphere_node16.model_z)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node16.objectName)) {
                                       sphere_node16.objectName = "16"
                                       sphere_node16.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node16.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name17
                               text: "  " + sphere_node17.objectName + "_" + sphere_node17.group_id
                               font.pixelSize: 62
                               color: sphere_node17.set_main ? "red":"black"
                           }
                           id: sphere_node17
                           objectName: "17"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node17.select_color
                               diffuseColor: sphere_node17.is_connected ? (sphere_node17.is_main ? "red" : (sphere_node17.group_id === 1 ? modelColor1 : (sphere_node17.group_id === 2 ? modelColor2 : (sphere_node17.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(17,1,sphere_node17)
                               // get_pos(sphere_node17);
                             //  screen_pos_to_world_pos(17,1,sphere_node17)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 17) {
                                       if (if_main_node(sphere_node17.objectName)) {sphere_node17.is_main = true;reset_main_name(sphere_node17.objectName,sysid)}
                                       sphere_node17.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node17.is_connected = true
                                       idpos_map[sysid] = [sphere_node17.model_x,sphere_node17.model_y,sphere_node17.model_z]
                                       modelmp[sysid] = sphere_node17
                                       swarm_send.store_airplane_group(sysid, sphere_node17.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node17.set_main,sphere_node17.group_id)
                                       else
                                           update_other_airplane(sphere_node17.set_main,sphere_node17.set_main,sphere_node17.group_id)
                                       console.log("pos17",sphere_node17.model_x,sphere_node17.model_y,sphere_node17.model_z)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node17.objectName)) {
                                       sphere_node17.objectName = "17"
                                       sphere_node17.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node17.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name18
                               text: "  " + sphere_node18.objectName + "_" + sphere_node18.group_id
                               font.pixelSize: 62
                               color: sphere_node18.set_main ? "red":"black"
                           }
                           id: sphere_node18
                           objectName: "18"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node18.select_color
                               diffuseColor: sphere_node18.is_connected ? (sphere_node18.is_main ? "red" : (sphere_node18.group_id === 1 ? modelColor1 : (sphere_node18.group_id === 2 ? modelColor2 : (sphere_node18.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(18,1,sphere_node18)
                               // get_pos(sphere_node18);
                           //    screen_pos_to_world_pos(18,1,sphere_node18)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 18) {
                                       if (if_main_node(sphere_node18.objectName)) {sphere_node18.is_main = true;reset_main_name(sphere_node18.objectName,sysid)}
                                       sphere_node18.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node18.is_connected = true
                                       idpos_map[sysid] = [sphere_node18.model_x,sphere_node18.model_y,sphere_node18.model_z]
                                       modelmp[sysid] = sphere_node18
                                       swarm_send.store_airplane_group(sysid, sphere_node18.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node18.set_main,sphere_node18.group_id)
                                       else
                                           update_other_airplane(sphere_node18.set_main,sphere_node18.set_main,sphere_node18.group_id)
                                       console.log("pos18",sphere_node18.model_x,sphere_node18.model_y,sphere_node18.model_z)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node18.objectName)) {
                                       sphere_node18.objectName = "18"
                                       sphere_node18.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node18.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name19
                               text: "  " + sphere_node19.objectName + "_" + sphere_node19.group_id
                               font.pixelSize: 62
                               color: sphere_node19.set_main ? "red":"black"
                           }
                           id: sphere_node19
                           objectName: "19"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node19.select_color
                               diffuseColor: sphere_node19.is_connected ? (sphere_node19.is_main ? "red" : (sphere_node19.group_id === 1 ? modelColor1 : (sphere_node19.group_id === 2 ? modelColor2 : (sphere_node19.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(19,1,sphere_node19)
                               // get_pos(sphere_node19);
                             //  screen_pos_to_world_pos(19,1,sphere_node19)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 19) {
                                       if (if_main_node(sphere_node19.objectName)) {sphere_node19.is_main = true;reset_main_name(sphere_node19.objectName,sysid)}
                                       sphere_node19.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node19.is_connected = true
                                       idpos_map[sysid] = [sphere_node19.model_x,sphere_node19.model_y,sphere_node19.model_z]
                                       modelmp[sysid] = sphere_node19
                                       swarm_send.store_airplane_group(sysid, sphere_node19.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node19.set_main,sphere_node19.group_id)
                                       else
                                           update_other_airplane(sphere_node19.set_main,sphere_node19.set_main,sphere_node19.group_id)
                                       console.log("pos19",sphere_node19.model_x,sphere_node19.model_y,sphere_node19.model_z)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node19.objectName)) {
                                       sphere_node19.objectName = "19"
                                       sphere_node19.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node19.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name20
                               text: "  " + sphere_node20.objectName + "_" + sphere_node20.group_id
                               font.pixelSize: 62
                               color: sphere_node20.set_main ? "red":"black"
                           }
                           id: sphere_node20
                           objectName: "20"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node20.select_color
                               diffuseColor: sphere_node20.is_connected ? (sphere_node20.is_main ? "red" : (sphere_node20.group_id === 1 ? modelColor1 : (sphere_node20.group_id === 2 ? modelColor2 : (sphere_node20.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(20,1,sphere_node20)
                               // get_pos(sphere_node20);
                             //  screen_pos_to_world_pos(20,1,sphere_node20)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 20) {
                                       if (if_main_node(sphere_node20.objectName)) {sphere_node20.is_main = true;reset_main_name(sphere_node20.objectName,sysid)}
                                       sphere_node20.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node20.is_connected = true
                                       idpos_map[sysid] = [sphere_node20.model_x,sphere_node20.model_y,sphere_node20.model_z]
                                       modelmp[sysid] = sphere_node20
                                       swarm_send.store_airplane_group(sysid, sphere_node20.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node20.set_main,sphere_node20.group_id)
                                       else
                                           update_other_airplane(sphere_node20.set_main,sphere_node20.set_main,sphere_node20.group_id)
                                       console.log("pos20",sphere_node20.model_x,sphere_node20.model_y,sphere_node20.model_z)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node20.objectName)) {
                                       sphere_node20.objectName = "20"
                                       sphere_node20.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node20.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name21
                               text: "  " + sphere_node21.objectName + "_" + sphere_node21.group_id
                               font.pixelSize: 62
                               color: sphere_node21.set_main ? "red":"black"
                           }
                           id: sphere_node21
                           objectName: "21"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node21.select_color
                               diffuseColor: sphere_node21.is_connected ? (sphere_node21.is_main ? "red" : (sphere_node21.group_id === 1 ? modelColor1 : (sphere_node21.group_id === 2 ? modelColor2 : (sphere_node21.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(21,1,sphere_node21)
                               // get_pos(sphere_node21);
                              // screen_pos_to_world_pos(21,1,sphere_node21)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 21) {
                                       if (if_main_node(sphere_node21.objectName)) {sphere_node21.is_main = true;reset_main_name(sphere_node21.objectName,sysid)}
                                       sphere_node21.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node21.is_connected = true
                                       idpos_map[sysid] = [sphere_node21.model_x,sphere_node21.model_y,sphere_node21.model_z]
                                       modelmp[sysid] = sphere_node21
                                       swarm_send.store_airplane_group(sysid, sphere_node21.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node21.set_main,sphere_node21.group_id)
                                       else
                                           update_other_airplane(sphere_node21.set_main,sphere_node21.set_main,sphere_node21.group_id)
                                       console.log("pos21",sphere_node21.model_x,sphere_node21.model_y,sphere_node21.model_z)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node21.objectName)) {
                                       sphere_node21.objectName = "21"
                                       sphere_node21.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node21.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name22
                               text: "  " + sphere_node22.objectName + "_" + sphere_node22.group_id
                               font.pixelSize: 62
                               color: sphere_node22.set_main ? "red":"black"
                           }
                           id: sphere_node22
                           objectName: "22"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               diffuseColor: sphere_node22.is_connected ? (sphere_node22.is_main ? "red" : (sphere_node22.group_id === 1 ? modelColor1 : (sphere_node22.group_id === 2 ? modelColor2 : (sphere_node22.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                               opacity: sphere_node22.select_color
                           }

                           Component.onCompleted: {
                               // mymove(22,1,sphere_node22)
                               // get_pos(sphere_node22);
                              // screen_pos_to_world_pos(22,1,sphere_node22)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 22) {
                                       if (if_main_node(sphere_node22.objectName)) {sphere_node22.is_main = true;reset_main_name(sphere_node22.objectName,sysid)}
                                       sphere_node22.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node22.is_connected = true
                                       idpos_map[sysid] = [sphere_node22.model_x,sphere_node22.model_y,sphere_node22.model_z]
                                       modelmp[sysid] = sphere_node22
                                       swarm_send.store_airplane_group(sysid, sphere_node22.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node22.set_main,sphere_node22.group_id)
                                       else
                                           update_other_airplane(sphere_node22.set_main,sphere_node22.set_main,sphere_node22.group_id)
                                       console.log("pos22",sphere_node22.model_x,sphere_node22.model_y,sphere_node22.model_z)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node22.objectName)) {
                                       sphere_node22.objectName = "22"
                                       sphere_node22.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node22.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name23
                               text: "  " + sphere_node23.objectName + "_" + sphere_node23.group_id
                               font.pixelSize: 62
                               color: sphere_node23.set_main ? "red":"black"
                           }
                           id: sphere_node23
                           objectName: "23"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               diffuseColor: sphere_node23.is_connected ? (sphere_node23.is_main ? "red" : (sphere_node23.group_id === 1 ? modelColor1 : (sphere_node23.group_id === 2 ? modelColor2 : (sphere_node23.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                               opacity: sphere_node23.select_color
                           }

                           Component.onCompleted: {
                               // mymove(23,1,sphere_node23)
                               // get_pos(sphere_node23);
                              // screen_pos_to_world_pos(23,1,sphere_node23)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 23) {
                                       if (if_main_node(sphere_node23.objectName)) {sphere_node23.is_main = true;reset_main_name(sphere_node23.objectName,sysid)}
                                       sphere_node23.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node23.is_connected = true
                                       idpos_map[sysid] = [sphere_node23.model_x,sphere_node23.model_y,sphere_node23.model_z]
                                       modelmp[sysid] = sphere_node23
                                       swarm_send.store_airplane_group(sysid, sphere_node23.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node23.set_main,sphere_node23.group_id)
                                       else
                                           update_other_airplane(sphere_node23.set_main,sphere_node23.set_main,sphere_node23.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node23.objectName)) {
                                       sphere_node23.objectName = "23"
                                       sphere_node23.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node23.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name24
                               text: "  " + sphere_node24.objectName + "_" + sphere_node24.group_id
                               font.pixelSize: 62
                               color: sphere_node24.set_main ? "red":"black"
                           }
                           id: sphere_node24
                           objectName: "24"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node24.select_color
                               diffuseColor: sphere_node24.is_connected ? (sphere_node24.is_main ? "red" : (sphere_node24.group_id === 1 ? modelColor1 : (sphere_node24.group_id === 2 ? modelColor2 : (sphere_node24.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(24,1,sphere_node24)
                               // get_pos(sphere_node24);
                             //  screen_pos_to_world_pos(24,1,sphere_node24)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 24) {
                                       if (if_main_node(sphere_node24.objectName)) {sphere_node24.is_main = true;reset_main_name(sphere_node24.objectName,sysid)}
                                       sphere_node24.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node24.is_connected = true
                                       idpos_map[sysid] = [sphere_node24.model_x,sphere_node24.model_y,sphere_node24.model_z]
                                       modelmp[sysid] = sphere_node24
                                       swarm_send.store_airplane_group(sysid, sphere_node24.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node24.set_main,sphere_node24.group_id)
                                       else
                                           update_other_airplane(sphere_node24.set_main,sphere_node24.set_main,sphere_node24.group_id)
                                   }

                                   console.log("qml 收到cpp的消息",n)
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node24.objectName)) {
                                       sphere_node24.objectName = "24"
                                       sphere_node24.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node24.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name25
                               text: "  " + sphere_node25.objectName + "_" + sphere_node25.group_id
                               font.pixelSize: 62
                               color: sphere_node25.set_main ? "red":"black"
                           }
                           id: sphere_node25
                           objectName: "25"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node25.select_color
                               diffuseColor: sphere_node25.is_connected ? (sphere_node25.is_main ? "red" : (sphere_node25.group_id === 1 ? modelColor1 : (sphere_node25.group_id === 2 ? modelColor2 : (sphere_node25.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(25,1,sphere_node25)
                               // get_pos(sphere_node25)

                             //  screen_pos_to_world_pos(25,1,sphere_node25)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 25) {
                                       if (if_main_node(sphere_node25.objectName)) {sphere_node25.is_main = true;reset_main_name(sphere_node25.objectName,sysid)}
                                       sphere_node25.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node25.is_connected = true
                                       idpos_map[sysid] = [sphere_node25.model_x,sphere_node25.model_y,sphere_node25.model_z]
                                       modelmp[sysid] = sphere_node25
                                       swarm_send.store_airplane_group(sysid, sphere_node25.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node25.set_main,sphere_node25.group_id)
                                       else
                                           update_other_airplane(sphere_node25.set_main,sphere_node25.set_main,sphere_node25.group_id)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node25.objectName)) {
                                       sphere_node25.objectName = "25"
                                       sphere_node25.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node25.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name26
                               text: "  " + sphere_node26.objectName + "_" + sphere_node26.group_id
                               font.pixelSize: 62
                               color: sphere_node26.set_main ? "red":"black"
                           }
                           id: sphere_node26
                           objectName: "26"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node26.select_color
                               diffuseColor: sphere_node26.is_connected ? (sphere_node26.is_main ? "red" : (sphere_node26.group_id === 1 ? modelColor1 : (sphere_node26.group_id === 2 ? modelColor2 : (sphere_node26.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(26,1,sphere_node26)
                               // get_pos(sphere_node26);
                            //   screen_pos_to_world_pos(26,1,sphere_node26)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 26) {
                                       if (if_main_node(sphere_node26.objectName)) {sphere_node26.is_main = true;reset_main_name(sphere_node26.objectName,sysid)}
                                       sphere_node26.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node26.is_connected = true
                                       idpos_map[sysid] = [sphere_node26.model_x,sphere_node26.model_y,sphere_node26.model_z]
                                       modelmp[sysid] = sphere_node26
                                       swarm_send.store_airplane_group(sysid, sphere_node26.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node26.set_main,sphere_node26.group_id)
                                       else
                                           update_other_airplane(sphere_node26.set_main,sphere_node26.set_main,sphere_node26.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node26.objectName)) {
                                       sphere_node26.objectName = "26"
                                       sphere_node26.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node26.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name27
                               text: "  " + sphere_node27.objectName + "_" + sphere_node27.group_id
                               font.pixelSize: 62
                               color: sphere_node27.set_main ? "red":"black"
                           }
                           id: sphere_node27
                           objectName: "27"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node27.select_color
                               diffuseColor: sphere_node27.is_connected ? (sphere_node27.is_main ? "red" : (sphere_node27.group_id === 1 ? modelColor1 : (sphere_node27.group_id === 2 ? modelColor2 : (sphere_node27.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(27,1,sphere_node27)
                               // get_pos(sphere_node27);
                             //  screen_pos_to_world_pos(27,1,sphere_node27)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 27) {
                                       if (if_main_node(sphere_node27.objectName)) {sphere_node27.is_main = true;reset_main_name(sphere_node27.objectName,sysid)}
                                       sphere_node27.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node27.is_connected = true
                                       idpos_map[sysid] = [sphere_node27.model_x,sphere_node27.model_y,sphere_node27.model_z]
                                       modelmp[sysid] = sphere_node27
                                       swarm_send.store_airplane_group(sysid, sphere_node27.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node27.set_main,sphere_node27.group_id)
                                       else
                                           update_other_airplane(sphere_node27.set_main,sphere_node27.set_main,sphere_node27.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node27.objectName)) {
                                       sphere_node27.objectName = "27"
                                       sphere_node27.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node27.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name28
                               text: "  " + sphere_node28.objectName + "_" + sphere_node28.group_id
                               font.pixelSize: 62
                               color: sphere_node28.set_main ? "red":"black"
                           }
                           id: sphere_node28
                           objectName: "28"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node28.select_color
                               diffuseColor: sphere_node28.is_connected ? (sphere_node28.is_main ? "red" : (sphere_node28.group_id === 1 ? modelColor1 : (sphere_node28.group_id === 2 ? modelColor2 : (sphere_node28.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(1,2,sphere_node28)
                               // get_pos(sphere_node28);
                             //  screen_pos_to_world_pos(1,2,sphere_node28)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 28) {
                                       if (if_main_node(sphere_node28.objectName)) {sphere_node28.is_main = true;reset_main_name(sphere_node28.objectName,sysid)}
                                       sphere_node28.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node28.is_connected = true
                                       idpos_map[sysid] = [sphere_node28.model_x,sphere_node28.model_y,sphere_node28.model_z]
                                       modelmp[sysid] = sphere_node28
                                       swarm_send.store_airplane_group(sysid, sphere_node28.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node28.set_main,sphere_node28.group_id)
                                       else
                                           update_other_airplane(sphere_node28.set_main,sphere_node28.set_main,sphere_node28.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node28.objectName)) {
                                       sphere_node28.objectName = "28"
                                       sphere_node28.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node28.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name29
                               text: "  " + sphere_node29.objectName + "_" + sphere_node29.group_id
                               font.pixelSize: 62
                               color: sphere_node29.set_main ? "red":"black"
                           }
                           id: sphere_node29
                           objectName: "29"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node29.select_color
                               diffuseColor: sphere_node29.is_connected ? (sphere_node29.is_main ? "red" : (sphere_node29.group_id === 1 ? modelColor1 : (sphere_node29.group_id === 2 ? modelColor2 : (sphere_node29.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(2,2,sphere_node29)
                               // get_pos(sphere_node29);
                              // screen_pos_to_world_pos(2,2,sphere_node29)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 29) {
                                       if (if_main_node(sphere_node29.objectName)) {sphere_node29.is_main = true;reset_main_name(sphere_node29.objectName,sysid)}
                                       sphere_node29.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node29.is_connected = true
                                       idpos_map[sysid] = [sphere_node29.model_x,sphere_node29.model_y,sphere_node29.model_z]
                                       modelmp[sysid] = sphere_node29
                                       swarm_send.store_airplane_group(sysid, sphere_node29.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node29.set_main,sphere_node29.group_id)
                                       else
                                           update_other_airplane(sphere_node29.set_main,sphere_node29.set_main,sphere_node29.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node29.objectName)) {
                                       sphere_node29.objectName = "29"
                                       sphere_node29.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node29.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name30
                               text: "  " + sphere_node30.objectName + "_" + sphere_node30.group_id
                               font.pixelSize: 62
                               color: sphere_node30.set_main ? "red":"black"
                           }
                           id: sphere_node30
                           objectName: "30"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node30.select_color
                               diffuseColor: sphere_node30.is_connected ? (sphere_node30.is_main ? "red" : (sphere_node30.group_id === 1 ? modelColor1 : (sphere_node30.group_id === 2 ? modelColor2 : (sphere_node30.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(3,2,sphere_node30)
                               // get_pos(sphere_node30);
                              // screen_pos_to_world_pos(3,2,sphere_node30)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 30) {
                                       if (if_main_node(sphere_node30.objectName)) {sphere_node30.is_main = true;reset_main_name(sphere_node30.objectName,sysid)}
                                       sphere_node30.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node30.is_connected = true
                                       idpos_map[sysid] = [sphere_node30.model_x,sphere_node30.model_y,sphere_node30.model_z]
                                       modelmp[sysid] = sphere_node30
                                       swarm_send.store_airplane_group(sysid, sphere_node30.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node30.set_main,sphere_node30.group_id)
                                       else
                                           update_other_airplane(sphere_node30.set_main,sphere_node30.set_main,sphere_node30.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node30.objectName)) {
                                       sphere_node30.objectName = "30"
                                       sphere_node30.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node30.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name31
                               text: "  " + sphere_node31.objectName + "_" + sphere_node31.group_id
                               font.pixelSize: 62
                               color: sphere_node31.set_main ? "red":"black"
                           }
                           id: sphere_node31
                           objectName: "31"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node31.select_color
                               diffuseColor: sphere_node31.is_connected ? (sphere_node31.is_main ? "red" : (sphere_node31.group_id === 1 ? modelColor1 : (sphere_node31.group_id === 2 ? modelColor2 : (sphere_node31.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(4,2,sphere_node31)
                               // get_pos(sphere_node31);
                              // screen_pos_to_world_pos(4,2,sphere_node31)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 31) {
                                       if (if_main_node(sphere_node31.objectName)) {sphere_node31.is_main = true;reset_main_name(sphere_node31.objectName,sysid)}
                                       sphere_node31.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node31.is_connected = true
                                       idpos_map[sysid] = [sphere_node31.model_x,sphere_node31.model_y,sphere_node31.model_z]
                                       modelmp[sysid] = sphere_node31
                                       swarm_send.store_airplane_group(sysid, sphere_node31.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node31.set_main,sphere_node31.group_id)
                                       else
                                           update_other_airplane(sphere_node31.set_main,sphere_node31.set_main,sphere_node31.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node31.objectName)) {
                                       sphere_node31.objectName = "31"
                                       sphere_node31.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node31.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name32
                               text: "  " + sphere_node32.objectName + "_" + sphere_node32.group_id
                               font.pixelSize: 62
                               color: sphere_node32.set_main ? "red":"black"
                           }
                           id: sphere_node32
                           objectName: "32"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node32.select_color
                               diffuseColor: sphere_node32.is_connected ? (sphere_node32.is_main ? "red" : (sphere_node32.group_id === 1 ? modelColor1 : (sphere_node32.group_id === 2 ? modelColor2 : (sphere_node32.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(5,2,sphere_node32)
                               // get_pos(sphere_node32);
                             //  screen_pos_to_world_pos(5,2,sphere_node32)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 32) {
                                       if (if_main_node(sphere_node32.objectName)) {sphere_node32.is_main = true;reset_main_name(sphere_node32.objectName,sysid)}
                                       sphere_node32.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node32.is_connected = true
                                       idpos_map[sysid] = [sphere_node32.model_x,sphere_node32.model_y,sphere_node32.model_z]
                                       modelmp[sysid] = sphere_node32
                                       swarm_send.store_airplane_group(sysid, sphere_node32.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node32.set_main,sphere_node32.group_id)
                                       else
                                           update_other_airplane(sphere_node32.set_main,sphere_node32.set_main,sphere_node32.group_id)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node32.objectName)) {
                                       sphere_node32.objectName = "32"
                                       sphere_node32.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node32.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name33
                               text: "  " + sphere_node33.objectName + "_" + sphere_node33.group_id
                               font.pixelSize: 62
                               color: sphere_node33.set_main ? "red":"black"
                           }
                           id: sphere_node33
                           objectName: "33"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node33.select_color
                               diffuseColor: sphere_node33.is_connected ? (sphere_node33.is_main ? "red" : (sphere_node33.group_id === 1 ? modelColor1 : (sphere_node33.group_id === 2 ? modelColor2 : (sphere_node33.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(6,2,sphere_node33)
                               // get_pos(sphere_node33);
                             //  screen_pos_to_world_pos(6,2,sphere_node33)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 33) {
                                       if (if_main_node(sphere_node33.objectName)) {sphere_node33.is_main = true;reset_main_name(sphere_node33.objectName,sysid)}
                                       sphere_node33.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node33.is_connected = true
                                       idpos_map[sysid] = [sphere_node33.model_x,sphere_node33.model_y,sphere_node33.model_z]
                                       modelmp[sysid] = sphere_node33
                                       swarm_send.store_airplane_group(sysid, sphere_node33.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node33.set_main,sphere_node33.group_id)
                                       else
                                           update_other_airplane(sphere_node33.set_main,sphere_node33.set_main,sphere_node33.group_id)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node33.objectName)) {
                                       sphere_node33.objectName = "33"
                                       sphere_node33.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node33.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name34
                               text: "  " + sphere_node34.objectName + "_" + sphere_node34.group_id
                               font.pixelSize: 62
                               color: sphere_node34.set_main ? "red":"black"
                           }
                           id: sphere_node34
                           objectName: "34"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node34.select_color
                               diffuseColor: sphere_node34.is_connected ? (sphere_node34.is_main ? "red" : (sphere_node34.group_id === 1 ? modelColor1 : (sphere_node34.group_id === 2 ? modelColor2 : (sphere_node34.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(7,2,sphere_node34)
                               // get_pos(sphere_node34);
                              // screen_pos_to_world_pos(7,2,sphere_node34)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 34) {
                                       if (if_main_node(sphere_node34.objectName)) {sphere_node34.is_main = true;reset_main_name(sphere_node34.objectName,sysid)}
                                       sphere_node34.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node34.is_connected = true
                                       idpos_map[sysid] = [sphere_node34.model_x,sphere_node34.model_y,sphere_node34.model_z]
                                       modelmp[sysid] = sphere_node34
                                       swarm_send.store_airplane_group(sysid, sphere_node34.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node34.set_main,sphere_node34.group_id)
                                       else
                                           update_other_airplane(sphere_node34.set_main,sphere_node34.set_main,sphere_node34.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node34.objectName)) {
                                       sphere_node34.objectName = "34"
                                       sphere_node34.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node34.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name35
                               text: "  " + sphere_node35.objectName + "_" + sphere_node35.group_id
                               font.pixelSize: 62
                               color: sphere_node35.set_main ? "red":"black"
                           }
                           id: sphere_node35
                           objectName: "35"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node35.select_color
                               diffuseColor: sphere_node35.is_connected ? (sphere_node35.is_main ? "red" : (sphere_node35.group_id === 1 ? modelColor1 : (sphere_node35.group_id === 2 ? modelColor2 : (sphere_node35.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(8,2,sphere_node35)
                               // get_pos(sphere_node35);
                              // screen_pos_to_world_pos(8,2,sphere_node35)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 35) {
                                       if (if_main_node(sphere_node35.objectName)) {sphere_node35.is_main = true;reset_main_name(sphere_node35.objectName,sysid)}
                                       sphere_node35.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node35.is_connected = true
                                       idpos_map[sysid] = [sphere_node35.model_x,sphere_node35.model_y,sphere_node35.model_z]
                                       modelmp[sysid] = sphere_node35
                                       swarm_send.store_airplane_group(sysid, sphere_node35.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node35.set_main,sphere_node35.group_id)
                                       else
                                           update_other_airplane(sphere_node35.set_main,sphere_node35.set_main,sphere_node35.group_id)
                                   }
                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node35.objectName)) {
                                       sphere_node35.objectName = "35"
                                       sphere_node35.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node35.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name36
                               text: "  " + sphere_node36.objectName + "_" + sphere_node36.group_id
                               font.pixelSize: 62
                               color: sphere_node36.set_main ? "red":"black"
                           }
                           id: sphere_node36
                           objectName: "36"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node36.select_color
                               diffuseColor: sphere_node36.is_connected ? (sphere_node36.is_main ? "red" : (sphere_node36.group_id === 1 ? modelColor1 : (sphere_node36.group_id === 2 ? modelColor2 : (sphere_node36.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(9,2,sphere_node36)
                               // get_pos(sphere_node36);
                             //  screen_pos_to_world_pos(9,2,sphere_node36)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 36) {
                                       if (if_main_node(sphere_node36.objectName)) {sphere_node36.is_main = true;reset_main_name(sphere_node36.objectName,sysid)}
                                       sphere_node36.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node36.is_connected = true
                                       idpos_map[sysid] = [sphere_node36.model_x,sphere_node36.model_y,sphere_node36.model_z]
                                       modelmp[sysid] = sphere_node36
                                       swarm_send.store_airplane_group(sysid, sphere_node36.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node36.set_main,sphere_node36.group_id)
                                       else
                                           update_other_airplane(sphere_node36.set_main,sphere_node36.set_main,sphere_node36.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node36.objectName)) {
                                       sphere_node36.objectName = "36"
                                       sphere_node36.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node36.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name37
                               text: "  " + sphere_node37.objectName + "_" + sphere_node37.group_id
                               font.pixelSize: 62
                               color: sphere_node37.set_main ? "red":"black"
                           }
                           id: sphere_node37
                           objectName: "37"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node37.select_color
                               diffuseColor: sphere_node37.is_connected ? (sphere_node37.is_main ? "red" : (sphere_node37.group_id === 1 ? modelColor1 : (sphere_node37.group_id === 2 ? modelColor2 : (sphere_node37.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(10,2,sphere_node37)
                               // get_pos(sphere_node37);
                              // screen_pos_to_world_pos(10,2,sphere_node37)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 37) {
                                       if (if_main_node(sphere_node37.objectName)) {sphere_node37.is_main = true;reset_main_name(sphere_node37.objectName,sysid)}
                                       sphere_node37.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node37.is_connected = true
                                       idpos_map[sysid] = [sphere_node37.model_x,sphere_node37.model_y,sphere_node37.model_z]
                                       modelmp[sysid] = sphere_node37
                                       swarm_send.store_airplane_group(sysid, sphere_node37.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node37.set_main,sphere_node37.group_id)
                                       else
                                           update_other_airplane(sphere_node37.set_main,sphere_node37.set_main,sphere_node37.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node37.objectName)) {
                                       sphere_node37.objectName = "37"
                                       sphere_node37.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node37.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name38
                               text: "  " + sphere_node38.objectName + "_" + sphere_node38.group_id
                               font.pixelSize: 62
                               color: sphere_node38.set_main ? "red":"black"
                           }
                           id: sphere_node38
                           objectName: "38"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node38.select_color
                               diffuseColor: sphere_node38.is_connected ? (sphere_node38.is_main ? "red" : (sphere_node38.group_id === 1 ? modelColor1 : (sphere_node38.group_id === 2 ? modelColor2 : (sphere_node38.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(11,2,sphere_node38)
                               // get_pos(sphere_node38);
                              // screen_pos_to_world_pos(11,2,sphere_node38)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 38) {
                                       if (if_main_node(sphere_node38.objectName)) {sphere_node38.is_main = true;reset_main_name(sphere_node38.objectName,sysid)}
                                       sphere_node38.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node38.is_connected = true
                                       idpos_map[sysid] = [sphere_node38.model_x,sphere_node38.model_y,sphere_node38.model_z]
                                       modelmp[sysid] = sphere_node38
                                       swarm_send.store_airplane_group(sysid, sphere_node38.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node38.set_main,sphere_node38.group_id)
                                       else
                                           update_other_airplane(sphere_node38.set_main,sphere_node38.set_main,sphere_node38.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node38.objectName)) {
                                       sphere_node38.objectName = "38"
                                       sphere_node38.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node38.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name39
                               text: "  " + sphere_node39.objectName + "_" + sphere_node39.group_id
                               font.pixelSize: 62
                               color: sphere_node39.set_main ? "red":"black"
                           }
                           id: sphere_node39
                           objectName: "39"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node39.select_color
                               diffuseColor: sphere_node39.is_connected ? (sphere_node39.is_main ? "red" : (sphere_node39.group_id === 1 ? modelColor1 : (sphere_node39.group_id === 2 ? modelColor2 : (sphere_node39.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(12,2,sphere_node39)
                               // get_pos(sphere_node39);
                             //  screen_pos_to_world_pos(12,2,sphere_node39)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 39) {
                                       if (if_main_node(sphere_node39.objectName)) {sphere_node39.is_main = true;reset_main_name(sphere_node39.objectName,sysid)}
                                       sphere_node39.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node39.is_connected = true
                                       idpos_map[sysid] = [sphere_node39.model_x,sphere_node39.model_y,sphere_node39.model_z]
                                       modelmp[sysid] = sphere_node39
                                       swarm_send.store_airplane_group(sysid, sphere_node39.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node39.set_main,sphere_node39.group_id)
                                       else
                                           update_other_airplane(sphere_node39.set_main,sphere_node39.set_main,sphere_node39.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node39.objectName)) {
                                       sphere_node39.objectName = "39"
                                       sphere_node39.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node39.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name40
                               text: "  " + sphere_node40.objectName + "_" + sphere_node40.group_id
                               font.pixelSize: 62
                               color: sphere_node40.set_main ? "red":"black"
                           }
                           id: sphere_node40
                           objectName: "40"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node40.select_color
                               diffuseColor: sphere_node40.is_connected ? (sphere_node40.is_main ? "red" : (sphere_node40.group_id === 1 ? modelColor1 : (sphere_node40.group_id === 2 ? modelColor2 : (sphere_node40.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(13,2,sphere_node40)
                               // get_pos(sphere_node40);
                             //  screen_pos_to_world_pos(13,2,sphere_node40)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 40) {
                                       if (if_main_node(sphere_node40.objectName)) {sphere_node40.is_main = true;reset_main_name(sphere_node40.objectName,sysid)}
                                       sphere_node40.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node40.is_connected = true
                                       idpos_map[sysid] = [sphere_node40.model_x,sphere_node40.model_y,sphere_node40.model_z]
                                       modelmp[sysid] = sphere_node40
                                       swarm_send.store_airplane_group(sysid, sphere_node40.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node40.set_main,sphere_node40.group_id)
                                       else
                                           update_other_airplane(sphere_node40.set_main,sphere_node40.set_main,sphere_node40.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node40.objectName)) {
                                       sphere_node40.objectName = "40"
                                       sphere_node40.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node40.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name41
                               text: "  " + sphere_node41.objectName + "_" + sphere_node41.group_id
                               font.pixelSize: 62
                               color: sphere_node41.set_main ? "red":"black"
                           }
                           id: sphere_node41
                           objectName: "41"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node41.select_color
                               diffuseColor: sphere_node41.is_connected ? (sphere_node41.is_main ? "red" : (sphere_node41.group_id === 1 ? modelColor1 : (sphere_node41.group_id === 2 ? modelColor2 : (sphere_node41.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(14,2,sphere_node41)
                               // get_pos(sphere_node41);
                              // screen_pos_to_world_pos(14,2,sphere_node41)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 41) {
                                       if (if_main_node(sphere_node41.objectName)) {sphere_node41.is_main = true;reset_main_name(sphere_node41.objectName,sysid)}
                                       sphere_node41.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node41.is_connected = true
                                       idpos_map[sysid] = [sphere_node41.model_x,sphere_node41.model_y,sphere_node41.model_z]
                                       modelmp[sysid] = sphere_node41
                                       swarm_send.store_airplane_group(sysid, sphere_node41.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node41.set_main,sphere_node41.group_id)
                                       else
                                           update_other_airplane(sphere_node41.set_main,sphere_node41.set_main,sphere_node41.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node41.objectName)) {
                                       sphere_node41.objectName = "41"
                                       sphere_node41.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node41.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name42
                               text: "  " + sphere_node42.objectName + "_" + sphere_node42.group_id
                               font.pixelSize: 62
                               color: sphere_node42.set_main ? "red":"black"
                           }
                           id: sphere_node42
                           objectName: "42"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node42.select_color
                               diffuseColor: sphere_node42.is_connected ? (sphere_node42.is_main ? "red" : (sphere_node42.group_id === 1 ? modelColor1 : (sphere_node42.group_id === 2 ? modelColor2 : (sphere_node42.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(15,2,sphere_node42)
                               // get_pos(sphere_node42);
                             //  screen_pos_to_world_pos(15,2,sphere_node42)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 42) {
                                       if (if_main_node(sphere_node42.objectName)) {sphere_node42.is_main = true;reset_main_name(sphere_node42.objectName,sysid)}
                                       sphere_node42.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node42.is_connected = true
                                       idpos_map[sysid] = [sphere_node42.model_x,sphere_node42.model_y,sphere_node42.model_z]
                                       modelmp[sysid] = sphere_node42
                                       swarm_send.store_airplane_group(sysid, sphere_node42.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node42.set_main,sphere_node42.group_id)
                                       else
                                           update_other_airplane(sphere_node42.set_main,sphere_node42.set_main,sphere_node42.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node42.objectName)) {
                                       sphere_node42.objectName = "42"
                                       sphere_node42.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node42.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name43
                               text: "  " + sphere_node43.objectName + "_" + sphere_node43.group_id
                               font.pixelSize: 62
                               color: sphere_node43.set_main ? "red":"black"
                           }
                           id: sphere_node43
                           objectName: "43"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node43.select_color
                               diffuseColor: sphere_node43.is_connected ? (sphere_node43.is_main ? "red" : (sphere_node43.group_id === 1 ? modelColor1 : (sphere_node43.group_id === 2 ? modelColor2 : (sphere_node43.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(16,2,sphere_node43)
                               // get_pos(sphere_node43);
                             //  screen_pos_to_world_pos(16,2,sphere_node43)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 43) {
                                       if (if_main_node(sphere_node43.objectName)) {sphere_node43.is_main = true;reset_main_name(sphere_node43.objectName,sysid)}
                                       sphere_node43.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node43.is_connected = true
                                       idpos_map[sysid] = [sphere_node43.model_x,sphere_node43.model_y,sphere_node43.model_z]
                                       modelmp[sysid] = sphere_node43
                                       swarm_send.store_airplane_group(sysid, sphere_node43.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node43.set_main,sphere_node43.group_id)
                                       else
                                           update_other_airplane(sphere_node43.set_main,sphere_node43.set_main,sphere_node43.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node43.objectName)) {
                                       sphere_node43.objectName = "43"
                                       sphere_node43.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node43.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name44
                               text: "  " + sphere_node44.objectName + "_" + sphere_node44.group_id
                               font.pixelSize: 62
                               color: sphere_node44.set_main ? "red":"black"
                           }
                           id: sphere_node44
                           objectName: "44"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node44.select_color
                               diffuseColor: sphere_node44.is_connected ? (sphere_node44.is_main ? "red" : (sphere_node44.group_id === 1 ? modelColor1 : (sphere_node44.group_id === 2 ? modelColor2 : (sphere_node44.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(17,2,sphere_node44)
                               // get_pos(sphere_node44);
                             //  screen_pos_to_world_pos(17,2,sphere_node44)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 44) {
                                       if (if_main_node(sphere_node44.objectName)) {sphere_node44.is_main = true;reset_main_name(sphere_node44.objectName,sysid)}
                                       sphere_node44.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node44.is_connected = true
                                       idpos_map[sysid] = [sphere_node44.model_x,sphere_node44.model_y,sphere_node44.model_z]
                                       modelmp[sysid] = sphere_node44
                                       swarm_send.store_airplane_group(sysid, sphere_node44.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node44.set_main,sphere_node44.group_id)
                                       else
                                           update_other_airplane(sphere_node44.set_main,sphere_node44.set_main,sphere_node44.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node44.objectName)) {
                                       sphere_node44.objectName = "44"
                                       sphere_node44.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node44.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name45
                               text: "  " + sphere_node45.objectName + "_" + sphere_node45.group_id
                               font.pixelSize: 62
                               color: sphere_node45.set_main ? "red":"black"
                           }
                           id: sphere_node45
                           objectName: "45"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node45.select_color
                               diffuseColor: sphere_node45.is_connected ? (sphere_node45.is_main ? "red" : (sphere_node45.group_id === 1 ? modelColor1 : (sphere_node45.group_id === 2 ? modelColor2 : (sphere_node45.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(18,2,sphere_node45)
                               // get_pos(sphere_node45);
                             //  screen_pos_to_world_pos(18,2,sphere_node45)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 45) {
                                       if (if_main_node(sphere_node45.objectName)) {sphere_node45.is_main = true;reset_main_name(sphere_node45.objectName,sysid)}
                                       sphere_node45.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node45.is_connected = true
                                       idpos_map[sysid] = [sphere_node45.model_x,sphere_node45.model_y,sphere_node45.model_z]
                                       modelmp[sysid] = sphere_node45
                                       swarm_send.store_airplane_group(sysid, sphere_node45.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node45.set_main,sphere_node45.group_id)
                                       else
                                           update_other_airplane(sphere_node45.set_main,sphere_node45.set_main,sphere_node45.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node45.objectName)) {
                                       sphere_node45.objectName = "45"
                                       sphere_node45.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node45.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name46
                               text: "  " + sphere_node46.objectName + "_" + sphere_node46.group_id
                               font.pixelSize: 62
                               color: sphere_node46.set_main ? "red":"black"
                           }
                           id: sphere_node46
                           objectName: "46"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node46.select_color
                               diffuseColor: sphere_node46.is_connected ? (sphere_node46.is_main ? "red" : (sphere_node46.group_id === 1 ? modelColor1 : (sphere_node46.group_id === 2 ? modelColor2 : (sphere_node46.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(19,2,sphere_node46)
                               // get_pos(sphere_node46);
                             //  screen_pos_to_world_pos(19,2,sphere_node46)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 46) {
                                       if (if_main_node(sphere_node46.objectName)) {sphere_node46.is_main = true;reset_main_name(sphere_node46.objectName,sysid)}
                                       sphere_node46.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node46.is_connected = true
                                       idpos_map[sysid] = [sphere_node46.model_x,sphere_node46.model_y,sphere_node46.model_z]
                                       modelmp[sysid] = sphere_node46
                                       swarm_send.store_airplane_group(sysid, sphere_node46.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node46.set_main,sphere_node46.group_id)
                                       else
                                           update_other_airplane(sphere_node46.set_main,sphere_node46.set_main,sphere_node46.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node46.objectName)) {
                                       sphere_node46.objectName = "46"
                                       sphere_node46.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node46.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name47
                               text: "  " + sphere_node47.objectName + "_" + sphere_node47.group_id
                               font.pixelSize: 62
                               color: sphere_node47.set_main ? "red":"black"
                           }
                           id: sphere_node47
                           objectName: "47"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node47.select_color
                               diffuseColor: sphere_node47.is_connected ? (sphere_node47.is_main ? "red" : (sphere_node47.group_id === 1 ? modelColor1 : (sphere_node47.group_id === 2 ? modelColor2 : (sphere_node47.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(20,2,sphere_node47)
                               // get_pos(sphere_node47);
                              // screen_pos_to_world_pos(20,2,sphere_node47)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 47) {
                                       if (if_main_node(sphere_node47.objectName)) {sphere_node47.is_main = true;reset_main_name(sphere_node47.objectName,sysid)}
                                       sphere_node47.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node47.is_connected = true
                                       idpos_map[sysid] = [sphere_node47.model_x,sphere_node47.model_y,sphere_node47.model_z]
                                       modelmp[sysid] = sphere_node47
                                       swarm_send.store_airplane_group(sysid, sphere_node47.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node47.set_main,sphere_node47.group_id)
                                       else
                                           update_other_airplane(sphere_node47.set_main,sphere_node47.set_main,sphere_node47.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node47.objectName)) {
                                       sphere_node47.objectName = "47"
                                       sphere_node47.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node47.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name48
                               text: "  " + sphere_node48.objectName + "_" + sphere_node48.group_id
                               font.pixelSize: 62
                               color: sphere_node48.set_main ? "red":"black"
                           }
                           id: sphere_node48
                           objectName: "48"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node48.select_color
                               diffuseColor: sphere_node48.is_connected ? (sphere_node48.is_main ? "red" : (sphere_node48.group_id === 1 ? modelColor1 : (sphere_node48.group_id === 2 ? modelColor2 : (sphere_node48.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(21,2,sphere_node48)
                               // get_pos(sphere_node48);
                             //  screen_pos_to_world_pos(21,2,sphere_node48)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 48) {
                                       if (if_main_node(sphere_node48.objectName)) {sphere_node48.is_main = true;reset_main_name(sphere_node48.objectName,sysid)}
                                       sphere_node48.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node48.is_connected = true
                                       idpos_map[sysid] = [sphere_node48.model_x,sphere_node48.model_y,sphere_node48.model_z]
                                       modelmp[sysid] = sphere_node48
                                       swarm_send.store_airplane_group(sysid, sphere_node48.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node48.set_main,sphere_node48.group_id)
                                       else
                                           update_other_airplane(sphere_node48.set_main,sphere_node48.set_main,sphere_node48.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node48.objectName)) {
                                       sphere_node48.objectName = "48"
                                       sphere_node48.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node48.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name49
                               text: "  " + sphere_node49.objectName + "_" + sphere_node49.group_id
                               font.pixelSize: 62
                               color: sphere_node49.set_main ? "red":"black"
                           }
                           id: sphere_node49
                           objectName: "49"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node49.select_color
                               diffuseColor: sphere_node49.is_connected ? (sphere_node49.is_main ? "red" : (sphere_node49.group_id === 1 ? modelColor1 : (sphere_node49.group_id === 2 ? modelColor2 : (sphere_node49.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(22,2,sphere_node49)
                               // get_pos(sphere_node49);
                             //  screen_pos_to_world_pos(22,2,sphere_node49)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 49) {
                                       if (if_main_node(sphere_node49.objectName)) {sphere_node49.is_main = true;reset_main_name(sphere_node49.objectName,sysid)}
                                       sphere_node49.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node49.is_connected = true
                                       idpos_map[sysid] = [sphere_node49.model_x,sphere_node49.model_y,sphere_node49.model_z]
                                       modelmp[sysid] = sphere_node49
                                       swarm_send.store_airplane_group(sysid, sphere_node49.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node49.set_main,sphere_node49.group_id)
                                       else
                                           update_other_airplane(sphere_node49.set_main,sphere_node49.set_main,sphere_node49.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node49.objectName)) {
                                       sphere_node49.objectName = "49"
                                       sphere_node49.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node49.group_id = 1
                                   }
                               }
                           }
                       }
                       Model {
                           Text {
                               id: name50
                               text: "  " + sphere_node50.objectName + "_" + sphere_node50.group_id
                               font.pixelSize: 62
                               color: sphere_node50.set_main ? "red":"black"
                           }
                           id: sphere_node50
                           objectName: "50"
                           source: "#Sphere"
                           pickable: true
                           visible: true
                           x: 0
                           z: 5
                           scale: Qt.vector3d(0.8,0.8,0.1)
                           property int model_x : 0
                           property int model_y : 0
                           property int model_z : 0
                           property int group_id: 1
                           property bool set_main: false
                           property bool is_main: false
                           property bool is_connected: false
                           property real select_color: 0.6
                           materials: DefaultMaterial {
                               opacity: sphere_node50.select_color
                               diffuseColor: sphere_node50.is_connected ? (sphere_node50.is_main ? "red" : (sphere_node50.group_id === 1 ? modelColor1 : (sphere_node50.group_id === 2 ? modelColor2 : (sphere_node50.group_id === 3 ? modelColor3 : modelColor4)))) : "#646566"
                           }

                           Component.onCompleted: {
                               // mymove(23,2,sphere_node50)
                               // get_pos(sphere_node50);
                            //   screen_pos_to_world_pos(23,2,sphere_node50)
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydatachanged(n,sysid) { // cpp发信号
                                   if (n === 50) {
                                       if (if_main_node(sphere_node50.objectName)) {sphere_node50.is_main = true;reset_main_name(sphere_node50.objectName,sysid)}
                                       sphere_node50.objectName = sysid
                                       _sysid_list.push(sysid)
                                     //  sphere_node8.pickable = true
                                       sphere_node50.is_connected = true
                                       idpos_map[sysid] = [sphere_node50.model_x,sphere_node50.model_y,sphere_node50.model_z]
                                       modelmp[sysid] = sphere_node50
                                       swarm_send.store_airplane_group(sysid, sphere_node50.group_id,false)
                                       if (n === Number(input_plan.text))
                                           update_other_airplane(2,sphere_node50.set_main,sphere_node50.group_id)
                                       else
                                           update_other_airplane(sphere_node50.set_main,sphere_node50.set_main,sphere_node50.group_id)
                                   }

                               }
                           }
                           Connections {
                               target: QGroundControl.multiVehicleManager  //cpp模块
                               function onMydata_disconnected(sysid) {
                                   if (sysid === Number(sphere_node50.objectName)) {
                                       sphere_node50.objectName = "50"
                                       sphere_node50.is_connected = false
                                       for (var vei = 0; vei < _sysid_list.length; vei++) {
                                           if (_sysid_list[vei] === sysid) {
                                               _sysid_list.splice(vei,1)
                                           }
                                       }
                                       delete idpos_map[sysid]
                                       delete modelmp[sysid]
                                       sphere_node50.group_id = 1
                                   }
                               }
                           }
                       }

                       MouseArea {
                           id: mouse_area
                           anchors.fill: parent
                           hoverEnabled: false
                           property var pickNode: null
                           property var lastpickNode: null
                           //鼠标和物体xy的偏移
                           property real xOffset: 0
                           property real yOffset: 0
                           property real zOffset: 0

                           acceptedButtons:Qt.LeftButton | Qt.RightButton
                           onPressed: {
                               var if_right = pressedButtons & Qt.RightButton

                               //获取点在View上的屏幕坐标
                 //              pick_screen.text = "(" + mouse.x + ", " + mouse.y + ")"  不用了
                               //pick取与该点射线路径相交的离最近的Model的信息，返回PickResult对象
                               //因为该模块一直在迭代，新的版本可以从PickResult对象获取更多的信息
                               //Qt6中还提供了pickAll获取与该射线相交的所有Model信息
                               var result = control.pick(mouse.x, mouse.y)
                               //目前只在点击时更新了pick物体的信息
                           //    Keys.rightPressed
                               if (result.objectHit) {
                                   pickNode = result.objectHit
                               //    pick_name.text = pickNode.objectName    改掉了  不用了
                                 //  pick_distance.text = result.distance.toFixed(2)
                               /*    pick_word.text = "("
                                           + result.scenePosition.x.toFixed(2) + ", "
                                           + result.scenePosition.y.toFixed(2) + ", "
                                           + result.scenePosition.z.toFixed(2) + ")" */

                                   var map_from = control.mapFrom3DScene(pickNode.scenePosition)
                                   //var map_to = control.mapTo3DScene(Qt.vector3d(mouse.x,mouse.y,map_from.z))

                                   xOffset = map_from.x - mouse.x
                                   yOffset = map_from.y - mouse.y
                                   zOffset = map_from.z

                                   if_release = true
                                   
                                   // 先更新位置信息，再更新相对坐标显示
                                   show_position(pickNode)
                                   updateRelativePosition(pickNode)
                                   
                                   // 多选操作
                                   if (ifpick) {
                                       select_merge.push(pickNode)
                                   }
                                   if (if_right) { // 可在onpressed里做筛选条件
                                       pickNode.select_color = 1
                                       select_merge.push(pickNode)
                                       console.log("right mouse",pickNode.objectName,pickNode.group_id)
                                   }
                               } else {
                                   pickNode = null
                             //      pick_name.text = "None"
                              //     pick_distance.text = " "
                             //      pick_word.text = " "
                                   
                                   // 清空相对坐标显示
                                   updateRelativePosition(null)

                                   for(var i = 0; i < select_merge.length; i++)
                                       select_merge[i].select_color = 0.6
                                   select_merge.length = 0
                               }

                           }
                          /* onReleased: {
                              // if (pickNode) {
                                //   send_all_airplane_pos()
                                   console.log("released")
                              // }
                           }*/
                           onPositionChanged: {
                               if(!mouse_area.containsMouse || !pickNode){
                                   return
                               }

                               show_position(pickNode)
                               updateRelativePosition(pickNode)  // 移动时更新相对坐标

                               if(if_release) { // 解决被挤开后再次回来碰撞问题
                                   var pos_temp = Qt.vector3d(mouse.x + xOffset, mouse.y + yOffset, zOffset);
                                   var map_to = control.mapTo3DScene(pos_temp)
                                   pickNode.x = map_to.x
                                   pickNode.y = map_to.y
                               }

                               var map_from_1 = control.mapFrom3DScene(pickNode.scenePosition) // 屏幕坐标
                               if (((map_from_1.x -20) % 40 <= 1 || (map_from_1.x -20) % 40 >= 39) && ((map_from_1.y-20) % 40 <= 1 || (map_from_1.y-20) % 40 >= 39)) {
                                   return
                               }
                         //      console.log(map_from_1.x,map_from_1.y)
                               var nu_x = (map_from_1.x-20) % 40 > 20 ? map_from_1.x + 40 - (map_from_1.x-20) % 40 : map_from_1.x - (map_from_1.x-20) % 40;
                               var nu_y = (map_from_1.y-20) % 40 > 20 ? map_from_1.y + 40 - (map_from_1.y-20) % 40 : map_from_1.y - (map_from_1.y-20) % 40;

                               var pos_temp_1 = Qt.vector3d(nu_x,nu_y, map_from_1.z);
                               var map_to_1 = control.mapTo3DScene(pos_temp_1) // 世界坐标
                               pickNode.x = map_to_1.x
                               pickNode.y = map_to_1.y

                               if(canv.visible === true) {// 左右
                                   var x = pickNode.is_connected ? idpos_map[Number(pickNode.objectName)][0]: pickNode.model_x
                                   var y = pickNode.is_connected ? idpos_map[Number(pickNode.objectName)][1]: pickNode.model_y
                                   if(mouse.x < (control.width ) / 2 && (2 === grp_pos_mp[pickNode.group_id])) {
                                       screen_pos_to_world_pos((control.width - 20) / 80,y,pickNode)
                                       if_release = false
                                   } else if (mouse.x > (control.width ) / 2 && (grp_pos_mp[pickNode.group_id] === 1)){
                                       screen_pos_to_world_pos((control.width - 20) / 80 - 1,y,pickNode)
                                       if_release = false
                                   }
                               }
                               if(canv2.visible === true) {// 左右
                                   var x2 = pickNode.is_connected ? idpos_map[Number(pickNode.objectName)][0]: pickNode.model_x
                                   var y2 = pickNode.is_connected ? idpos_map[Number(pickNode.objectName)][1]: pickNode.model_y
                                   if(mouse.x < (control.width ) / 2 && (grp_pos_mp[pickNode.group_id] === 4)) {
                                       screen_pos_to_world_pos((control.width - 20) / 80,y2,pickNode)
                                       if_release = false
                                   } else if (mouse.x > (control.width ) / 2 && (grp_pos_mp[pickNode.group_id] === 3)){
                                       screen_pos_to_world_pos((control.width - 20) / 80 - 1,y2,pickNode)
                                       if_release = false
                                   }
                               }
                               if(canv3.visible === true) {
                                   var x_1 = pickNode.is_connected ? idpos_map[Number(pickNode.objectName)][0]: pickNode.model_x
                                   var y_1 = pickNode.is_connected ? idpos_map[Number(pickNode.objectName)][1]: pickNode.model_y
                                   if(mouse.y < (control.height ) / 2 && (grp_pos_mp[pickNode.group_id] === 3)) {
                                       screen_pos_to_world_pos(x_1,(control.height - 20) / 80,pickNode)
                                       if_release = false
                                   } else if (mouse.y > (control.height ) / 2 && (grp_pos_mp[pickNode.group_id] === 1)){
                                       screen_pos_to_world_pos(x_1,(control.height - 20) / 80 - 2,pickNode)
                                       if_release = false
                                   }
                               }
                               if(canv4.visible === true) {
                                   var x_2 = pickNode.is_connected ? idpos_map[Number(pickNode.objectName)][0]: pickNode.model_x
                                   var y_2 = pickNode.is_connected ? idpos_map[Number(pickNode.objectName)][1]: pickNode.model_y
                                   if(mouse.y < (control.height ) / 2 && (grp_pos_mp[pickNode.group_id] === 4)) {
                                       screen_pos_to_world_pos(x_2,(control.height - 20) / 80,pickNode)
                                       if_release = false
                                   } else if (mouse.y > (control.height ) / 2 && (grp_pos_mp[pickNode.group_id] === 2)){
                                       screen_pos_to_world_pos(x_2,(control.height - 20) / 80 - 2,pickNode)
                                       if_release = false
                                   }
                               }

                               if(pickNode.is_connected) {
                                   lastIntx = myIntx
                                   lastInty = myInty
                                   get_pos(pickNode)
                                   idpos_map[Number(pickNode.objectName)][0] = myIntx
                                   idpos_map[Number(pickNode.objectName)][1] = myInty
                                   if  (will_crush(pickNode)) { //先移动再检测，检测无问题继续，碰撞时移动至上次位置
                                       screen_pos_to_world_pos(lastIntx,lastInty,pickNode)
                                       if_release = false

                                       idpos_map[Number(pickNode.objectName)][0] = lastIntx
                                       idpos_map[Number(pickNode.objectName)][1] = lastInty
                                   }

                                   send_all_airplane_pos(pickNode.group_id,0)
                               }
                             //  console.log(pickNode.objectName,idpos_map[Number(pickNode.objectName)][0],idpos_map[Number(pickNode.objectName)][1])
                              /* if (pickNode.is_connected) {
                                   get_pos(pickNode)
                                   idpos_map[Number(pickNode.objectName)][0] = myIntx
                                   idpos_map[Number(pickNode.objectName)][1] = myInty
                                   send_all_airplane_pos()
                               }*/
                           }

                           WheelHandler {
                               onWheel:{//鼠标滚轮滚动
                                   if(event.angleDelta.y > 0){
                                    //   perspective_camera.z -= 15//相机靠近，放大
                                   } else {
                                   //    perspective_camera.z += 15//相机远离，缩小
                                   }
                               }
                           }


                           DragHandler {
                               property bool _isRotating: false
                               property point _lastPose;
                               id: cameraRotationDragHandler
                               target: null
                           }

                       }

                       // Myvehicle {
                       //        id: myVehicleInstance // 给车辆实例一个唯一ID，方便后续引用
                       //        position: Qt.vector3d(0, 0, 0) // 初始位置
                       //       // eulerRotation.x: 90 // 模型姿态调整
                       //        scale: Qt.vector3d(3, 3, 3) // 缩放
                       //        Text {
                       //            id: name1
                       //           // text:"  56"
                       //            text: "  " + myVehicleInstance.objectName + "_" + myVehicleInstance.group_id
                       //            font.pixelSize: 62
                       //            rotation:0
                       //          //  color: myVehicleInstance.set_main ? "red":"black"
                       //            y:-100
                       //        }

                       //        objectName: "4"
                       //        x: 0
                       //        y: 1
                       //        //scale: Qt.vector3d(1,1,0.1)
                       //        z: 5
                       //        visible: true
                       //       function loadModel() {
                       //            visible = true;
                       //        }

                       //        property int model_x : 0
                       //        property int model_y : 0
                       //        property int model_z : 0
                       //        property int group_id: 1
                       //        property bool is_main: false
                       //        property bool set_main: false
                       //        property bool is_connected: false
                       //        property real select_color: 0.6
                       //    }

                       // Myvehicle {
                       //        id: myVehicleInstance2 // 给车辆实例一个唯一ID，方便后续引用
                       //        position: Qt.vector3d(100, 100, 0) // 初始位置
                       //        eulerRotation.x: 90 // 模型姿态调整
                       //        scale: Qt.vector3d(3, 3, 3) // 缩放
                       //        Text {
                       //            id: name2
                       //            text: "  " + myVehicleInstance2.objectName + "_" + myVehicleInstance2.group_id
                       //            font.pixelSize: 62
                       //          //  color: myVehicleInstance.set_main ? "red":"black"
                       //            y:-100
                       //        }
                       //        objectName: "5"
                       //        x: 0
                       //        y: 1
                       //        //scale: Qt.vector3d(1,1,0.1)
                       //        z: 5
                       //        visible: true
                       //       function loadModel() {
                       //            visible = true;
                       //        }

                       //        property int model_x : 0
                       //        property int model_y : 0
                       //        property int model_z : 0
                       //        property int group_id: 1
                       //        property bool is_main: false
                       //        property bool set_main: false
                       //        property bool is_connected: false
                       //        property real select_color: 0.6
                       //    }

                // 您的3D场景内容保持不变
                // ......

                }

            Keys.onPressed: {
                console.log("keys")
                if (Number(event.key) === Number(Qt.Key_Control)) {
                    ifpick = true
                    console.log("ctrl")
                }
                event.accepted = true // 标记事件已被处理
            }
            Keys.onReleased: {
                // 当释放任意键时触发
                root.ifpick = false
                event.accepted = true // 标记事件已被处理
            }
            //}
        }

        // 控制面板
        Rectangle {
            id: controlPanel
            width: 245
            anchors {
                top: topBar.bottom
                bottom: bottomBar.top
                left: parent.left
                margins: 12
            }
            color: root.panelColor
            radius: 8
            border.color: "#4c566a"
            border.width: 1

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 15
                spacing: 20

                // // 分组管理标题 不需要了
                // Label {
                //     text: "分组管理"
                //     font.bold: true
                //     font.pixelSize: 18
                //     color: accentColor
                //     Layout.alignment: Qt.AlignHCenter
                // }



                // 分组设置
                GroupBox {
                    title: "分组设置"
                    Layout.fillWidth: true
                    background: Rectangle {
                        color: "transparent"
                        border.color: "#4c566a"
                        radius: 4
                    }

                    label: Label {
                        text: parent.title
                        color: primaryColor
                        font.bold: true
                        leftPadding: 5
                    }

                    GridLayout {
                        columns: 2
                        columnSpacing: 10
                        rowSpacing: 15
                        anchors.fill: parent

                        Label {
                            text: "当前分组:"
                            color: root.textColor
                        }
                        Label {
                            text: group_num
                            font.bold: true
                            color: secondaryColor
                        }

                        // 目标分组行：标签 + 输入框 + 分组按钮
                        Label {
                            text: "目标分组:"
                            color: textColor
                        }
                        Row {
                            spacing: 8  // 输入框与按钮的间距
                            CustomTextField {
                                id: input3
                                text: "1"
                                Layout.fillWidth: true
                                validator: IntValidator { bottom: 1; top: 4 }
                            }
                            CustomButton {
                                text: "分组"
                                color: root.primaryColor
                                onClicked: {
                                    if (Number(input3.text) > Number(input_plan.text)) {
                                        popup.open()
                                    } else { // 执行分组
                                        if (input3.text === "1") {   //
                                            canv.visible = false
                                            canv2.visible = false
                                            canv3.visible = false
                                            canv4.visible = false
                                            for (var i1 = 0; i1 <plan_arr.length; i1++) {
                                                plan_arr[i1].group_id = 1
                                                if(plan_arr[i1].is_connected)swarm_send.store_airplane_group(plan_arr[i1].objectName, plan_arr[i1].group_id, true)
                                            }
                                            main_node_name.length = 1
                                            main_node_name[0] = plan_arr[0].objectName
                                           /* swarm_send.set_main_airplane(main_node_name[0], modelmp[main_node_name[0]].group_id,  未考虑连接后再设置的情况
                                                              idpos_map[main_node_name[0]][0],
                                                              idpos_map[main_node_name[0]][1],
                                                              idpos_map[main_node_name[0]][2])*/
                                            set_main_name(plan_arr[0])
                                            if(plan_arr[0].is_connected === true)set_main_color(plan_arr[0].objectName)
                                            send_all_airplane_pos(1,0)

                                            grp_pos_mp[1] = 1
                                            group_num = 1
                                        } else if (input3.text === "2") {//1、改groupid 2、设主机（同组的其他 视情况 设为从机）


                                            var j = 0
                                            for (var i = 0; i < plan_arr.length; i++) {
                                                if (i < plan_arr.length / 2) {  // 条件不适用
                                                    plan_arr[i].group_id = 1 // 把前一半变成第1组   保证和设置主机的group-1对应
                                                    j = i
                                                } else {
                                                    plan_arr[i].group_id = 2
                                                }
                                                if(plan_arr[i].is_connected)swarm_send.store_airplane_group(plan_arr[i].objectName, plan_arr[i].group_id, true) //
                                            }
                                            main_node_name.length = 2
                                            for (var n0 = 0; n0 < plan_arr.length / 2; n0++) {
                                                if (if_main_node(plan_arr[n0].objectName)) { // 用set_main判断
                                                    main_node_name[0] = plan_arr[n0].objectName
                                                    set_main_name(plan_arr[n0])
                                                    if(plan_arr[n0].is_connected === true)set_main_color(plan_arr[n0].objectName)
                                                    break;
                                                }

                                                main_node_name[0] = plan_arr[n0].objectName
                                                set_main_name(plan_arr[n0])
                                                if(plan_arr[n0].is_connected === true)set_main_color(plan_arr[n0].objectName)
                                            }

                                            for (var n1 = j + 1; n1 < plan_arr.length; n1++) {
                                                if (if_main_node(plan_arr[n1].objectName)) {
                                                    main_node_name[1] = plan_arr[n1].objectName
                                                    set_main_name(plan_arr[n1])
                                                    if(plan_arr[n1].is_connected === true)set_main_color(plan_arr[n1].objectName)
                                                    break;
                                                }
                                                main_node_name[1] = plan_arr[n1].objectName
                                                set_main_name(plan_arr[n1])
                                                if(plan_arr[n1].is_connected === true)set_main_color(plan_arr[n1].objectName)
                                            }/*
                if (modelmp[main_node_name[0]].is_connected === true)
                                            swarm_send.set_main_airplane(main_node_name[0], modelmp[main_node_name[0]].group_id,
                                                              idpos_map[main_node_name[0]][0],
                                                              idpos_map[main_node_name[0]][1],
                                                              idpos_map[main_node_name[0]][2])
                if (modelmp[main_node_name[1]].is_connected === true)
                                            swarm_send.set_main_airplane(main_node_name[1], modelmp[main_node_name[1]].group_id,
                                                              idpos_map[main_node_name[1]][0],
                                                              idpos_map[main_node_name[1]][1],
                                                              idpos_map[main_node_name[1]][2])
                */
                                        /*
                                            var j = 0
                                            for (var i = 0; i < _sysid_list.length; i++) {
                                                if (i < _sysid_list.length / 2) {
                                                    modelmp[_sysid_list[i]].group_id = 1 // 把前一半变成第1组   保证和设置主机的group-1对应
                                                    j = i
                                                } else {
                                                     modelmp[_sysid_list[i]].group_id = 2
                                                }
                                                swarm_send.store_airplane_group(_sysid_list[i], modelmp[_sysid_list[i]].group_id, true)
                                            }
                                            main_node_name.length = 2
                                            for (var n0 = 0; n0 < _sysid_list.length / 2; n0++) {
                                                if (if_main_node(modelmp[_sysid_list[n0]].objectName)) {
                                                    main_node_name[0] = modelmp[_sysid_list[n0]].objectName
                                                    set_main_color(modelmp[_sysid_list[n0]].objectName)
                                                    break;
                                                }

                                                main_node_name[0] = modelmp[_sysid_list[n0]].objectName
                                                set_main_color(modelmp[_sysid_list[n0]].objectName)
                                            }
                                            for (var n1 = j + 1; n1 < _sysid_list.length; n1++) {
                                                if (if_main_node(modelmp[_sysid_list[n1]].objectName)) {
                                                    main_node_name[1] = modelmp[_sysid_list[n1]].objectName
                                                    set_main_color(modelmp[_sysid_list[n1]].objectName)
                                                    break;
                                                }
                                                main_node_name[1] = modelmp[_sysid_list[n1]].objectName
                                                set_main_color(modelmp[_sysid_list[n1]].objectName)
                                            }
                                            swarm_send.set_main_airplane(main_node_name[0], modelmp[main_node_name[0]].group_id,
                                                              idpos_map[main_node_name[0]][0],
                                                              idpos_map[main_node_name[0]][1],
                                                              idpos_map[main_node_name[0]][2])
                                            swarm_send.set_main_airplane(main_node_name[1], modelmp[main_node_name[1]].group_id,
                                                              idpos_map[main_node_name[1]][0],
                                                              idpos_map[main_node_name[1]][1],
                                                              idpos_map[main_node_name[1]][2])*/
                                            send_all_airplane_pos(2,0)

                                            group_num = 2
                                            hasset_map[2] = 0
                                            for(i = 0; i < main_node_name.length;i++) {
                                                for(j = 0;j < plan_arr.length;j++){
                                                    if(plan_arr[j].objectName === main_node_name[i]) {

                                                        grp_pos_mp[plan_arr[j].group_id] = i + 1 // 要的是grp  不是name
                                                    }
                                                }
                                            }
                                            canv.visible = true
                                            canv2.visible = true
                                            canv3.visible = false
                                            canv4.visible = false
                                            // 强制重新绘制需要显示的Canvas
                                            canv.requestPaint()
                                            canv2.requestPaint()

                                            move_model(1,2)
                                            move_model(2,2)
                                        } else if (input3.text === "3") {
                                            console.log("设置分组3的Canvas可见性");
                                            canv.visible = true
                                            canv4.visible = true
                                            canv2.visible = true
                                            canv3.visible = true
                                            // 强制重新绘制Canvas
                                            canv.requestPaint()
                                            canv2.requestPaint()
                                            canv3.requestPaint()
                                            canv4.requestPaint()
                                            console.log("Canvas状态:", canv.visible, canv2.visible, canv3.visible, canv4.visible);
                                            for (var k = 0; k < plan_arr.length; k++) {
                                                if (k < plan_arr.length / 3) {
                                                    plan_arr[k].group_id = 1
                                                    i = k
                                                } else if (k >= plan_arr.length / 3 && k < plan_arr.length * 2 / 3) {
                                                    plan_arr[k].group_id = 2
                                                    j = k
                                                } else {
                                                    plan_arr[k].group_id = 3
                                                }
                                                if(plan_arr[k].is_connected)swarm_send.store_airplane_group(plan_arr[k].objectName, plan_arr[k].group_id, true)
                                            }
                                            main_node_name.length = 3

                                            for (var mn3_1 = 0; mn3_1 < plan_arr.length / 3; mn3_1++) {
                                                if (if_main_node(plan_arr[mn3_1].objectName)) {
                                                    main_node_name[0] = plan_arr[mn3_1].objectName
                                                    set_main_name(plan_arr[mn3_1])
                                                    if(plan_arr[mn3_1].is_connected === true)set_main_color(plan_arr[mn3_1].objectName)
                                                    break;
                                                }

                                                main_node_name[0] = plan_arr[mn3_1].objectName
                                                set_main_name(plan_arr[mn3_1])
                                                if(plan_arr[mn3_1].is_connected === true)set_main_color(plan_arr[mn3_1].objectName)
                                            }

                                            for (var mn3_2 = i + 1; mn3_2 < plan_arr.length * 2 / 3; mn3_2++) {
                                                if (if_main_node(plan_arr[mn3_2].objectName)) {
                                                    main_node_name[1] = plan_arr[mn3_2].objectName
                                                    set_main_name(plan_arr[mn3_2])
                                                    if(plan_arr[mn3_2].is_connected === true)set_main_color(plan_arr[mn3_2].objectName)
                                                    break;
                                                }
                                                main_node_name[1] = plan_arr[mn3_2].objectName
                                                set_main_name(plan_arr[mn3_2])
                                                if(plan_arr[mn3_2].is_connected === true)set_main_color(plan_arr[mn3_2].objectName)
                                            }

                                            for (var mn3_3 = j + 1; mn3_3 < plan_arr.length; mn3_3++) {
                                                if (if_main_node(plan_arr[mn3_3].objectName)) {
                                                    main_node_name[2] = plan_arr[mn3_3].objectName
                                                    set_main_name(plan_arr[mn3_3])
                                                    if(plan_arr[mn3_3].is_connected === true)set_main_color(plan_arr[mn3_3].objectName)
                                                    break;
                                                }
                                                main_node_name[2] = plan_arr[mn3_3].objectName
                                                set_main_name(plan_arr[mn3_3])
                                                if(plan_arr[mn3_3].is_connected === true)set_main_color(plan_arr[mn3_3].objectName)
                                            }

                                         /*   swarm_send.set_main_airplane(main_node_name[0], modelmp[main_node_name[0]].group_id,
                                                              idpos_map[main_node_name[0]][0],
                                                              idpos_map[main_node_name[0]][1],
                                                              idpos_map[main_node_name[0]][2])
                                            swarm_send.set_main_airplane(main_node_name[1], modelmp[main_node_name[1]].group_id,
                                                              idpos_map[main_node_name[1]][0],
                                                              idpos_map[main_node_name[1]][1],
                                                              idpos_map[main_node_name[1]][2])
                                            swarm_send.set_main_airplane(main_node_name[2], modelmp[main_node_name[2]].group_id,
                                                              idpos_map[main_node_name[2]][0],
                                                              idpos_map[main_node_name[2]][1],
                                                              idpos_map[main_node_name[2]][2])*/
                                            send_all_airplane_pos(3,0)

                                            group_num = 3
                                            for(i = 0; i < main_node_name.length;i++) {
                                                for(j = 0;j < plan_arr.length;j++){
                                                    if(plan_arr[j].objectName === main_node_name[i]) {

                                                        grp_pos_mp[plan_arr[j].group_id] = i + 1 // 要的是grp  不是name
                                                    }
                                                }
                                            }
                                            move_model(1,3)
                                            move_model(2,3)
                                            move_model(3,3)
                                        } else if (input3.text === "4") {
                                            canv.visible = true
                                            canv4.visible = true
                                            canv2.visible = true
                                            canv3.visible = true
                                            // 强制重新绘制Canvas
                                            canv.requestPaint()
                                            canv2.requestPaint()
                                            canv3.requestPaint()
                                            canv4.requestPaint()
                                            
                                            // 初始化边界索引
                                            var bound1 = Math.floor(plan_arr.length / 4);
                                            var bound2 = Math.floor(plan_arr.length / 2);
                                            var bound3 = Math.floor(plan_arr.length * 3 / 4);
                                            
                                            for (var l = 0; l < plan_arr.length; l++) {
                                                if (l < bound1) {
                                                    plan_arr[l].group_id = 1
                                                } else if (l < bound2) {
                                                    plan_arr[l].group_id = 2
                                                } else if (l < bound3) {
                                                    plan_arr[l].group_id = 3
                                                } else {
                                                    plan_arr[l].group_id = 4
                                                }
                                                if(plan_arr[l].is_connected)swarm_send.store_airplane_group(plan_arr[l].objectName, plan_arr[l].group_id, true)
                                            }
                                            main_node_name.length = 4
                                            main_node_name[0] = ""
                                            main_node_name[1] = ""
                                            main_node_name[2] = ""
                                            main_node_name[3] = ""

                                            // 为每组设置主机
                                            for (var mn4_1 = 0; mn4_1 < bound1 && mn4_1 < plan_arr.length; mn4_1++) {
                                                if (if_main_node(plan_arr[mn4_1].objectName)) {
                                                    main_node_name[0] = plan_arr[mn4_1].objectName
                                                    set_main_name(plan_arr[mn4_1])
                                                    if(plan_arr[mn4_1].is_connected === true)set_main_color(plan_arr[mn4_1].objectName)
                                                    break;
                                                }
                                                main_node_name[0] = plan_arr[mn4_1].objectName
                                                set_main_name(plan_arr[mn4_1])
                                                if(plan_arr[mn4_1].is_connected === true)set_main_color(plan_arr[mn4_1].objectName)
                                            }

                                            for (var mn4_2 = bound1; mn4_2 < bound2 && mn4_2 < plan_arr.length; mn4_2++) {
                                                if (if_main_node(plan_arr[mn4_2].objectName)) {
                                                    main_node_name[1] = plan_arr[mn4_2].objectName
                                                    set_main_name(plan_arr[mn4_2])
                                                    if(plan_arr[mn4_2].is_connected === true)set_main_color(plan_arr[mn4_2].objectName)
                                                    break;
                                                }
                                                main_node_name[1] = plan_arr[mn4_2].objectName
                                                set_main_name(plan_arr[mn4_2])
                                                if(plan_arr[mn4_2].is_connected === true)set_main_color(plan_arr[mn4_2].objectName)
                                            }

                                            for (var mn4_3 = bound2; mn4_3 < bound3 && mn4_3 < plan_arr.length; mn4_3++) {
                                                if (if_main_node(plan_arr[mn4_3].objectName)) {
                                                    main_node_name[2] = plan_arr[mn4_3].objectName
                                                    set_main_name(plan_arr[mn4_3])
                                                    if(plan_arr[mn4_3].is_connected === true)set_main_color(plan_arr[mn4_3].objectName)
                                                    break;
                                                }
                                                main_node_name[2] = plan_arr[mn4_3].objectName
                                                set_main_name(plan_arr[mn4_3])
                                                if(plan_arr[mn4_3].is_connected === true)set_main_color(plan_arr[mn4_3].objectName)
                                            }
                                            for (var mn4_4 = bound3; mn4_4 < plan_arr.length; mn4_4++) {
                                                if (if_main_node(plan_arr[mn4_4].objectName)) {
                                                    main_node_name[3] = plan_arr[mn4_4].objectName
                                                    set_main_name(plan_arr[mn4_4])
                                                    if(plan_arr[mn4_4].is_connected === true)set_main_color(plan_arr[mn4_4].objectName)
                                                    break;
                                                }
                                                main_node_name[3] = plan_arr[mn4_4].objectName
                                                set_main_name(plan_arr[mn4_4])
                                                if(plan_arr[mn4_4].is_connected === true){set_main_color(plan_arr[mn4_4].objectName)}
                                            }
                /*
                                            swarm_send.set_main_airplane(main_node_name[0], modelmp[main_node_name[0]].group_id,
                                                              idpos_map[main_node_name[0]][0],
                                                              idpos_map[main_node_name[0]][1],
                                                              idpos_map[main_node_name[0]][2])
                                            swarm_send.set_main_airplane(main_node_name[1], modelmp[main_node_name[1]].group_id,
                                                              idpos_map[main_node_name[1]][0],
                                                              idpos_map[main_node_name[1]][1],
                                                              idpos_map[main_node_name[1]][2])
                                            swarm_send.set_main_airplane(main_node_name[2], modelmp[main_node_name[2]].group_id,
                                                              idpos_map[main_node_name[2]][0],
                                                              idpos_map[main_node_name[2]][1],
                                                              idpos_map[main_node_name[2]][2])
                                            swarm_send.set_main_airplane(main_node_name[3], modelmp[main_node_name[3]].group_id,
                                                              idpos_map[main_node_name[3]][0],
                                                              idpos_map[main_node_name[3]][1],
                                                              idpos_map[main_node_name[3]][2])*/
                                            send_all_airplane_pos(4,0) // 四组全更新
                                            group_num = 4

                                            // 设置位置映射
                                            grp_pos_mp[1] = 1
                                            grp_pos_mp[2] = 2
                                            grp_pos_mp[3] = 3
                                            grp_pos_mp[4] = 4
                                            
                                            move_model(1,4)
                                            move_model(2,4)
                                            move_model(3,4)
                                            move_model(4,4)
                                        }
                                    }
                                    updateGroupCounts()  // 更新各组数量显示
                                }
                            }
                        }

                        // 更改组号行：标签 + 输入框 + 更换分组按钮
                        Label {
                            text: "更改组号:"
                            color: textColor
                        }
                        Row {
                            spacing: 8  // 输入框与按钮的间距
                            CustomTextField {
                                id: input_change
                                text: "1"
                                Layout.fillWidth: true
                                validator: IntValidator { bottom: 1; top: 4 }
                            }
                            CustomButton {
                                text: "更换分组"
                                color: secondaryColor
                                onClicked: {
                                    if (mouse_area.pickNode === null) {
                                        group_popup2.open()
                                        return
                                    }
                                    if (group_num >= input_change.text && input_change.text > 0 &&// 需要把连接功能放开
                                            mouse_area.pickNode.set_main !== true) {
                                        mouse_area.pickNode.group_id = Number(input_change.text)

                                        display_changed_pos(mouse_area.pickNode.group_id) //需要考虑更换后组内是否还有剩余，若无剩余  grp_pos_mp 消除

                                        if(hasset_map[mouse_area.pickNode.group_id]===1){
                                            swarm_send.store_airplane_group(Number(mouse_area.pickNode.objectName), modelmp[Number(mouse_area.pickNode.objectName)].group_id, true)


                                            send_all_airplane_pos(mouse_area.pickNode.group_id,0)
                                        }
                                        updateGroupCounts()  // 更新各组数量显示

                                    } else {
                                        group_popup.open()
                                    }
                                }
                            }
                        }


                        // 独立分组操作
                        Label {
                            text: "独立分组:"
                            color: textColor
                        }
                        Row {
                            spacing: 4

                            CustomButton {
                                text: "独立"
                                width: 35
                                color: root.accentColor
                                onClicked: {
                                    // 检查输入的组号是否有效
                                    var targetGroupId = Number(input_independent_group.text);
                                    if (targetGroupId < 1 || targetGroupId > 4) {
                                        console.log("目标组号必须在1-4之间");
                                        return;
                                    }

                                    // 检查是否有选中的模型
                                    if(select_merge.length === 0) {
                                        console.log("请先右键选择要独立分组的模型");
                                        return;
                                    }

                                    // 检查目标组号是否已经存在
                                    var targetGroupExists = false;
                                    for(var check = 0; check < plan_arr.length; check++) {
                                        if(plan_arr[check].group_id === targetGroupId) {
                                            targetGroupExists = true;
                                            break;
                                        }
                                    }

                                    if(targetGroupExists) {
                                        console.log("目标组号", targetGroupId, "已存在，请选择其他组号");
                                        return;
                                    }

                                    // 统计当前实际有模型的组数
                                    var actualGroupCount = 0;
                                    var existingGroups = {};
                                    for(var cnt = 0; cnt < plan_arr.length; cnt++) {
                                        if(!existingGroups[plan_arr[cnt].group_id]) {
                                            existingGroups[plan_arr[cnt].group_id] = true;
                                            actualGroupCount++;
                                        }
                                    }
                                    
                                    // 如果实际组数已经达到4组，不能再分组
                                    if(actualGroupCount >= 4) {
                                        select_merge.length = 0;
                                        for(var i1 = 0; i1 < select_merge.length; i1++) {
                                            select_merge[i1].select_color = 0.6;
                                        }
                                        devide_grp_pop.open();
                                        return;
                                    }

                                    // 将选中的模型分配到目标组
                                    for(var i = 0; i < select_merge.length; i++) {
                                        select_merge[i].group_id = targetGroupId;
                                        select_merge[i].select_color = 0.6;

                                        if(select_merge[i].is_connected === true) {
                                            swarm_send.store_airplane_group(select_merge[i].objectName, select_merge[i].group_id, true);
                                        }
                                    }

                                    // 确保main_node_name数组有足够的长度
                                    while(main_node_name.length < targetGroupId) {
                                        main_node_name.push("");
                                    }

                                    // 设置目标组的主机
                                    main_node_name[targetGroupId - 1] = select_merge[0].objectName;
                                    select_merge[0].set_main = 1;

                                    if(select_merge[0].is_connected === true) {
                                        swarm_send.set_main_airplane(main_node_name[targetGroupId - 1], select_merge[0].group_id,
                                                          0, 0, 0);
                                        select_merge[0].is_main = true;
                                        set_main_color(select_merge[0]);
                                    }

                                    // 更新组数为实际组数+1（新增的组）
                                    var newGroupNum = actualGroupCount + 1;
                                    
                                    // 收集当前所有被占用的屏幕位置
                                    var usedPositions = {};
                                    for(var grpKey in grp_pos_mp) {
                                        var grpId = Number(grpKey);
                                        // 检查这个组是否真的有模型
                                        var grpHasModels = false;
                                        for(var chk = 0; chk < plan_arr.length; chk++) {
                                            if(plan_arr[chk].group_id === grpId) {
                                                grpHasModels = true;
                                                break;
                                            }
                                        }
                                        if(grpHasModels && grp_pos_mp[grpKey] > 0) {
                                            usedPositions[grp_pos_mp[grpKey]] = grpId;
                                            console.log("位置", grp_pos_mp[grpKey], "被组", grpId, "占用");
                                        }
                                    }
                                    
                                    // 为新组分配一个空闲的屏幕位置
                                    // 位置1=左上, 2=右上, 3=左下, 4=右下
                                    // 优先分配与组号相同的位置（如第4组优先放位置4）
                                    var newPos = 0;
                                    if(!usedPositions[targetGroupId] && targetGroupId >= 1 && targetGroupId <= 4) {
                                        // 优先使用与组号相同的位置
                                        newPos = targetGroupId;
                                        grp_pos_mp[targetGroupId] = newPos;
                                        console.log("为组", targetGroupId, "分配对应位置", newPos);
                                    } else {
                                        // 否则找一个空闲的位置
                                        for(var pos = 1; pos <= 4; pos++) {
                                            if(!usedPositions[pos]) {
                                                newPos = pos;
                                                grp_pos_mp[targetGroupId] = pos;
                                                console.log("为组", targetGroupId, "分配空闲位置", pos);
                                                break;
                                            }
                                        }
                                    }
                                    
                                    // 设置分屏线可见性
                                    if(newGroupNum >= 3) {
                                        canv.visible = true;
                                        canv2.visible = true;
                                        canv3.visible = true;
                                        canv4.visible = true;
                                        canv.requestPaint();
                                        canv2.requestPaint();
                                        canv3.requestPaint();
                                        canv4.requestPaint();
                                    } else if(newGroupNum === 2) {
                                        canv.visible = true;
                                        canv2.visible = true;
                                        canv3.visible = false;
                                        canv4.visible = false;
                                        canv.requestPaint();
                                        canv2.requestPaint();
                                    }
                                    
                                    group_num = newGroupNum;
                                    
                                    // 将新组的模型移动到分配的屏幕位置，并检测碰撞
                                    if(newPos > 0) {
                                        moveGroupToPositionWithCollision(targetGroupId, newPos);
                                    }
                                    
                                    if(select_merge[0].is_connected === true) {
                                        send_all_airplane_pos(targetGroupId, 0);
                                    }

                                    select_merge.length = 0;

                                    console.log("独立分组完成，目标组号:", targetGroupId, "当前组数:", group_num);
                                    console.log("grp_pos_mp:", JSON.stringify(grp_pos_mp));
                                    updateGroupCounts()  // 更新各组数量显示
                                }
                            }
                            Label {
                                text: "到"
                                color: textColor
                                anchors.verticalCenter: parent.verticalCenter
                            }
                            CustomTextField {
                                id: input_independent_group
                                text: "2"
                                width: 30
                                validator: IntValidator { bottom: 1; top: 4 }
                            }
                            Label {
                                text: "组"
                                color: root.textColor
                                anchors.verticalCenter: parent.verticalCenter
                            }
                            CustomButton {
                                text: "合并"
                                width: 35
                                color: secondaryColor
                                onClicked: {
                                    // 需要选中至少2个主机才能合并
                                    if(select_merge.length < 2) {
                                        console.log("请选择至少2个主机进行合并");
                                        return;
                                    }
                                    
                                    // 检查所有选中的是否都是主机
                                    for(var c = 0; c < select_merge.length; c++) {
                                        console.log("检查模型:", select_merge[c].objectName, "set_main:", select_merge[c].set_main, "类型:", typeof select_merge[c].set_main);
                                        // 兼容 1, true, "1" 等情况
                                        if(select_merge[c].set_main != 1 && select_merge[c].set_main !== true) {
                                            console.log("只能选择主机进行合并, 模型", select_merge[c].objectName, "不是主机");
                                            return;
                                        }
                                    }
                                    
                                    var targetGroup = select_merge[0].group_id;  // 目标组（第一个选中的主机所在组）
                                    var targetPos = grp_pos_mp[targetGroup];  // 目标组的屏幕位置
                                    console.log("合并到组:", targetGroup, "位置:", targetPos);
                                    
                                    // 找到目标组当前占用的最大X坐标，用于放置被合并的模型
                                    var targetMaxX = 0;
                                    for(var t = 0; t < plan_arr.length; t++) {
                                        if(plan_arr[t].group_id === targetGroup) {
                                            if(plan_arr[t].model_x > targetMaxX) {
                                                targetMaxX = plan_arr[t].model_x;
                                            }
                                        }
                                    }
                                    console.log("目标组最大X坐标:", targetMaxX);
                                    
                                    // 处理被合并的组
                                    for(var h = 1; h < select_merge.length; h++) {
                                        var sourceGroup = select_merge[h].group_id;
                                        var sourcePos = grp_pos_mp[sourceGroup];  // 被合并组的屏幕位置
                                        console.log("合并组", sourceGroup, "位置", sourcePos, "到组", targetGroup, "位置", targetPos);
                                        
                                        // 收集被合并组的所有模型，并找到它们的最小X坐标
                                        var sourceModels = [];
                                        var sourceMinX = 9999;
                                        var sourceMinY = 9999;
                                        for(var j = 0; j < plan_arr.length; j++) {
                                            if(plan_arr[j].group_id === sourceGroup) {
                                                sourceModels.push(plan_arr[j]);
                                                if(plan_arr[j].model_x < sourceMinX) {
                                                    sourceMinX = plan_arr[j].model_x;
                                                }
                                                if(plan_arr[j].model_y < sourceMinY) {
                                                    sourceMinY = plan_arr[j].model_y;
                                                }
                                            }
                                        }
                                        console.log("被合并组模型数:", sourceModels.length, "最小X:", sourceMinX, "最小Y:", sourceMinY);
                                        
                                        // 计算偏移量：将被合并组平移到目标组右侧
                                        // 新位置X = 目标组最大X + 1，Y保持相对位置
                                        var offsetX = (targetMaxX + 1) - sourceMinX;
                                        // 找到目标组的最小Y，让被合并组Y对齐
                                        var targetMinY = 9999;
                                        for(var ty = 0; ty < plan_arr.length; ty++) {
                                            if(plan_arr[ty].group_id === targetGroup) {
                                                if(plan_arr[ty].model_y < targetMinY) {
                                                    targetMinY = plan_arr[ty].model_y;
                                                }
                                            }
                                        }
                                        var offsetY = targetMinY - sourceMinY;
                                        console.log("平移偏移量: offsetX=", offsetX, "offsetY=", offsetY);
                                        
                                        // 平移被合并组的所有模型（保持原队形）
                                        for(var k = 0; k < sourceModels.length; k++) {
                                            var model = sourceModels[k];
                                            var newX = model.model_x + offsetX;
                                            var newY = model.model_y + offsetY;
                                            console.log("平移模型", model.objectName, "从", model.model_x, model.model_y, "到", newX, newY);
                                            
                                            // 使用screen_pos_to_world_pos更新模型位置
                                            screen_pos_to_world_pos(newX, newY, model);
                                            
                                            // 更新组别
                                            model.group_id = targetGroup;
                                            model.set_main = 0;
                                            model.is_main = false;
                                            if(model.is_connected) {
                                                swarm_send.store_airplane_group(model.objectName, targetGroup, true, true);
                                            }
                                            
                                            // 更新targetMaxX以便下一组合并时使用
                                            if(newX > targetMaxX) {
                                                targetMaxX = newX;
                                            }
                                        }
                                        
                                        // 清除被合并组的主机信息
                                        main_node_name[sourceGroup - 1] = 0;
                                        
                                        // 清除被合并组的位置映射
                                        grp_pos_mp[sourceGroup] = 0;
                                        delete grp_pos_mp[sourceGroup];
                                        
                                        // 被合并的主机也变成从机
                                        select_merge[h].set_main = 0;
                                        select_merge[h].is_main = false;
                                        select_merge[h].select_color = 0.6;
                                    }
                                    
                                    select_merge[0].select_color = 0.6;
                                    
                                    // 更新组数
                                    group_num = group_num - select_merge.length + 1;
                                    
                                    // 根据剩余组的实际位置更新分屏线可见性
                                    // 位置: 1=左上, 2=右上, 3=左下, 4=右下
                                    // canv+canv2 = 垂直线| (分隔左右)
                                    // canv3+canv4 = 水平线— (分隔上下)
                                    var hasLeft = false;   // 左边有组 (位置1或3)
                                    var hasRight = false;  // 右边有组 (位置2或4)
                                    var hasTop = false;    // 上边有组 (位置1或2)
                                    var hasBottom = false; // 下边有组 (位置3或4)
                                    
                                    for(var grpKey in grp_pos_mp) {
                                        var pos = grp_pos_mp[grpKey];
                                        if(pos === 1) { hasLeft = true; hasTop = true; }
                                        if(pos === 2) { hasRight = true; hasTop = true; }
                                        if(pos === 3) { hasLeft = true; hasBottom = true; }
                                        if(pos === 4) { hasRight = true; hasBottom = true; }
                                    }
                                    
                                    // 垂直线：只有左右都有组时才显示
                                    var showVertical = hasLeft && hasRight;
                                    // 水平线：只有上下都有组时才显示
                                    var showHorizontal = hasTop && hasBottom;
                                    
                                    canv.visible = showVertical;   // 垂直线上半
                                    canv2.visible = showVertical;  // 垂直线下半
                                    canv3.visible = showHorizontal; // 水平线左半
                                    canv4.visible = showHorizontal; // 水平线右半
                                    
                                    if(showVertical) {
                                        canv.requestPaint();
                                        canv2.requestPaint();
                                    }
                                    if(showHorizontal) {
                                        canv3.requestPaint();
                                        canv4.requestPaint();
                                    }
                                    
                                    console.log("分界线状态: 垂直=", showVertical, "水平=", showHorizontal);
                                    
                                    if(select_merge[0].is_connected === true) {
                                        send_all_airplane_pos(targetGroup, 0);
                                    }
                                    
                                    select_merge.length = 0;
                                    updateGroupCounts();  // 更新各组数量显示
                                    console.log("合并完成，当前组数:", group_num);
                                }
                            }
                        }
                    }
                }

                // 队形设置
                GroupBox {
                    title: "队形设置"
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    background: Rectangle {
                        color: "transparent"
                        border.color: "#4c566a"
                        radius: 4
                    }

                    label: Label {
                        text: parent.title
                        color: root.primaryColor
                        font.bold: true
                        leftPadding: 5
                    }

                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 15

                        RowLayout {
                            spacing: 10
                            Label {
                                text: "第"
                                color: textColor
                            }
                            CustomTextField {
                                id: input4
                                text: "1"
                                width: 40
                                validator: IntValidator { bottom: 1; top: 4 }
                            }
                            Label {
                                text: "组"
                                color: textColor
                            }
                        }

                        CustomComboBox {
                            id: comboBox
                            Layout.fillWidth: true
                            model: ["队形变换", "东西一字形", "南北一字形", "三角队形", "正方队形", "菱形队形", "圆形队形"]
                            onActivated: {
                                // 保留原有功能
                                if (currentText === "东西一字形") {
                                    // 处理东西一字形
                                    stright_line_swarm()
                                } else if(currentText === "南北一字形") {
                                    // 处理南北一字形
                                    stright_NS_line_swarm()
                                } else if(currentText === "三角队形") {
                                    // 处理三角队形
                                    triangle_swarm()
                                } else if (currentText === "正方队形") {
                                    // 处理正方队形
                                    rectangle_swarm()
                                } else if (currentText === "菱形队形") {
                                    // 处理菱形队形
                                    diamond_swarm()
                                } else if (currentText === "圆形队形") {
                                    // 处理圆形队形
                                    circle_swarm()
                                }
                            }
                        }
                    }
                }

                //   新增
                GroupBox {
                    title: "参数设置"
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    background: Rectangle {
                        color: "transparent"
                        border.color: "#4c566a"
                        radius: 4
                    }

                    label: Label {
                        text: parent.title
                        color: primaryColor
                        font.bold: true
                        leftPadding: 5
                    }

                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 8

                        RowLayout {
                            spacing: 8
                            Label {
                                text: "间距:"
                                color: root.textColor
                                font.pixelSize: 12
                            }
                            CustomTextField {
                                id: input5
                                text: "1"
                                width: 30
                                validator: IntValidator { bottom: 1; top: 99 }
                                onTextChanged: {
                                    // 间距变化时更新相对坐标显示
                                    if (mouse_area.pickNode) {
                                        updateRelativePosition(mouse_area.pickNode);
                                    }
                                    // 间距变化时发送数据给飞控
                                    if (text !== "" && Number(text) > 0) {
                                        for (var g = 1; g <= group_num; g++) {
                                            send_all_airplane_pos(g, 0);
                                        }
                                    }
                                }
                            }
                            Label {
                                text: "米"
                                color: textColor
                                font.pixelSize: 12
                            }
                            Item { width: 10 }
                            Label {
                                text: "相对主机:" + (relativeMainName !== "" ? relativeMainName : "-")
                                color: "#88c0d0"
                                font.pixelSize: 11
                            }
                        }

                        RowLayout {
                            spacing: 8
                            Label {
                                text: "高度:"
                                color: textColor
                                font.pixelSize: 12
                            }
                            CustomTextField {
                                id: input6
                                text: "0"
                                width: 30
                                validator: IntValidator { bottom: -99; top: 99 }
                            }
                            Label {
                                text: "米"
                                color: textColor
                                font.pixelSize: 12
                            }
                            Item { width: 10 }
                            Label {
                                text: "东" + relativeEast
                                color: "#a3be8c"
                                font.pixelSize: 11
                            }
                            Label {
                                text: "北" + relativeNorth
                                color: "#a3be8c"
                                font.pixelSize: 11
                            }
                            Label {
                                text: "上" + relativeAlt
                                color: "#a3be8c"
                                font.pixelSize: 11
                            }
                        }
                        
                        RowLayout {
                        CustomButton {
                            text: "设置高度"
                            Layout.columnSpan: 2
                            Layout.fillWidth: true
                            color: root.primaryColor
                            onClicked: {
                                if (!mouse_area.pickNode) {
                                    return
                                }
                                mouse_area.pickNode.model_z = Number(input6.text)
                                if (mouse_area.pickNode.is_connected) {
                                    idpos_map[mouse_area.pickNode.objectName][2] = Number(input6.text)
                                }

                                show_position(mouse_area.pickNode) // 界面显示数据
                                updateRelativePosition(mouse_area.pickNode)  // 更新相对坐标
                                send_all_airplane_pos(mouse_area.pickNode.group_id,0)
                            }
                        }
                        CustomButton {
                            text: "选定主机"
                            Layout.columnSpan: 2
                            Layout.fillWidth: true
                            color: root.primaryColor
                            onClicked: {
                                if (mouse_area.pickNode === null) {
                                    return
                                }
                                mouse_area.pickNode.set_main = 1   // 需对同组其他主机进行排他

                                main_node_name[mouse_area.pickNode.group_id - 1] = mouse_area.pickNode.objectName
                                set_main_name(mouse_area.pickNode)
                                updateRelativePosition(mouse_area.pickNode)  // 更新相对坐标
                                    if (!mouse_area.pickNode.is_connected) return
                                    mouse_area.pickNode.is_main = true
                                  //  swarm_send.set_main_airplane(main_node_name[node.group_id - 1], node.group_id, 0, 0, 0)
                                    set_main_behavior(mouse_area.pickNode,1)  //要发位置

                            }
                        }
                        }
                    }
                }
                //  起飞  降落
                GroupBox {
                    title: "起降命令"
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    background: Rectangle {
                        color: "transparent"
                        border.color: "#4c566a"
                        radius: 4
                    }

                    label: Label {
                        text: parent.title
                        color: primaryColor
                        font.bold: true
                        leftPadding: 5
                    }

                    RowLayout {
                        anchors.fill: parent
                        spacing: 15

                        CustomButton {
                            text: "起飞"
                            Layout.columnSpan: 2
                            Layout.fillWidth: true
                            color: "#03DE6D"
                            onClicked: {
                                test_mavlink._sendcom(1,0,0,0,0)
                            }
                        }
                        CustomButton {
                            text: "降落"
                            Layout.columnSpan: 2
                            Layout.fillWidth: true
                            color: root.dangerColor
                            onClicked: {
                                test_mavlink._sendcom(0,0,1,0,0)
                            }
                        }
                    }
                }

            }
        }

        // 底部状态栏
        Rectangle {
            id: bottomBar
            width: parent.width
            height: 40
            anchors.bottom: parent.bottom
            color: root.panelColor
            radius: 8

            RowLayout {
                anchors.fill: parent
                anchors.margins: 10

                Label {
                    text: "就绪"
                    color: root.secondaryColor
                    font.pixelSize: 14
                }

                Item { Layout.fillWidth: true }

                // Label {
                //     text: "无人机数量: 12"
                //     color: textColor
                //     font.pixelSize: 14
                // }
            }
        }
    }

    // ========== 自定义控件 ==========

    // 自定义按钮
    component CustomButton : Button {
        property color color: primaryColor
        width: 80
        height: 36
        contentItem: Text {
            text: parent.text
            color: textColor
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 14
            font.bold: true
        }
        background: Rectangle {
            color: parent.down ? Qt.darker(parent.color, 1.2) :
                   parent.hovered ? Qt.lighter(parent.color, 1.2) : parent.color
            radius: 6
            border.width: 1
            border.color: Qt.lighter(parent.color, 1.5)
        }
    }
    component RadiusButton : Button {
        id: roundButton
        property color color: "#4CAF50" // 临时用具体颜色测试
        width: 20
        height: 20

        // 确保按钮保持固定尺寸，不受布局影响
        Layout.preferredWidth: 20
        Layout.preferredHeight: 20
        Layout.minimumWidth: 20
        Layout.minimumHeight: 20
        Layout.maximumWidth: 20
        Layout.maximumHeight: 20

        background: Rectangle {
            width: 20
            height: 20
            radius: 10  // 固定半径为10，确保完美圆形
            color: roundButton.down ? Qt.darker(roundButton.color, 1.2) :
                   roundButton.hovered ? Qt.lighter(roundButton.color, 1.2) : roundButton.color
            border.width: 1
            border.color: Qt.lighter(roundButton.color, 1.5)
        }
    }

    // 自定义文本框
    component CustomTextField : TextField {
        id: textField
        height: 36
        color: textColor
        font.pixelSize: 14
        selectionColor: primaryColor
        selectedTextColor: textColor
        placeholderTextColor: "#7f8c8d"

        background: Rectangle {
            color: controlColor
            radius: 6
            border.color: textField.activeFocus ? primaryColor : "#4c566a"
            border.width: 1
        }

        // 输入验证
        validator: IntValidator { bottom: 0; top: 999 }

        // 限制只能输入数字
        inputMethodHints: Qt.ImhDigitsOnly
    }

    // 自定义下拉框
    component CustomComboBox : ComboBox {
        id: comboControl
        height: 36
        contentItem: Text {
            text: comboControl.displayText
            color: textColor
            font.pixelSize: 14
            horizontalAlignment: Text.AlignLeft
            verticalAlignment: Text.AlignVCenter
            leftPadding: 10
        }
        background: Rectangle {
            color: controlColor
            radius: 6
            border.color: comboControl.activeFocus ? primaryColor : "#4c566a"
            border.width: 1
        }
        popup: Popup {
            y: comboControl.height
            width: comboControl.width
            implicitHeight: contentItem.implicitHeight
            padding: 1

            contentItem: ListView {
                clip: true
                implicitHeight: contentHeight
                model: comboControl.popup.visible ? comboControl.delegateModel : null
                currentIndex: comboControl.highlightedIndex

                ScrollIndicator.vertical: ScrollIndicator {}
            }

            background: Rectangle {
                color: controlColor
                border.color: primaryColor
                radius: 6
            }
        }

        delegate: ItemDelegate {
            width: comboControl.width
            contentItem: Text {
                text: modelData
                color: textColor
                font.pixelSize: 14
                elide: Text.ElideRight
                verticalAlignment: Text.AlignVCenter
                leftPadding: 10
            }
            background: Rectangle {
                color: highlighted ? primaryColor : "transparent"
                radius: 4
            }
            highlighted: comboControl.highlightedIndex === index
        }

        indicator: Canvas {
            width: 10
            height: 6
            // 调整位置：靠右、垂直居中，右侧留12像素边距
            anchors.right: parent.right
            anchors.verticalCenter: parent.verticalCenter
            anchors.rightMargin: 12  // 用 rightMargin 代替 rightPadding，设置右侧边距

            onPaint: {
                var ctx = getContext("2d");
                ctx.resetTransform();
                ctx.clearRect(0, 0, width, height);
                ctx.fillStyle = "white";
                ctx.beginPath();
                ctx.moveTo(0, 0);
                ctx.lineTo(width, 0);
                ctx.lineTo(width / 2, height);
                ctx.closePath();
                ctx.fill();
            }
        }
    }

    // ========== 弹窗组件 ==========

    // 警告弹窗
    Popup {
        id: popup
        anchors.centerIn: Overlay.overlay
        width: 320
        height: 160
        modal: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside

        background: Rectangle {
            color: "#3b4252"
            radius: 12
            border.color: dangerColor
            border.width: 2
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 20
            spacing: 20

            Label {
                text: "⚠️ 警告"
                font.bold: true
                font.pixelSize: 18
                color: dangerColor
                Layout.alignment: Qt.AlignHCenter
            }

            Label {
                text: "超出机架数量范围，请输入合理值!"
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
                Layout.fillWidth: true
                color: textColor
            }

            CustomButton {
                text: "确定"
                Layout.alignment: Qt.AlignHCenter
                onClicked: popup.close()
                color: dangerColor
            }
        }
    }

    // 其他弹窗 (保持原有功能)
    Popup {
        id: group_popup
        // 内容与popup类似，可复用样式
        anchors.centerIn: Overlay.overlay
        width: 320
        height: 160
        modal: true
        background: Rectangle {
            color: "#3b4252"
            radius: 12
            border.color: accentColor
            border.width: 2
        }
        contentItem: ColumnLayout {
            anchors.fill: parent
            anchors.margins: 20
            spacing: 20
            Label {
                text: "⚠️ 提示"
                font.bold: true
                font.pixelSize: 18
                color: accentColor
                Layout.alignment: Qt.AlignHCenter
            }
            Label {
                text: "此分组数值目前状态不可用!"
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
                Layout.fillWidth: true
                color: textColor
            }
            CustomButton {
                text: "确定"
                Layout.alignment: Qt.AlignHCenter
                onClicked: group_popup.close()
                color: primaryColor
            }
        }
    }

    Popup {
        id: group_popup2
        // 内容与popup类似，可复用样式
    }

    Popup {
        id: group_popu4
        anchors.centerIn: Overlay.overlay
        width: 320
        height: 160
        modal: true
        background: Rectangle {
            color: "#3b4252"
            radius: 12
            border.color: dangerColor
            border.width: 2
        }
        contentItem: ColumnLayout {
            anchors.fill: parent
            anchors.margins: 20
            spacing: 20
            Label {
                text: "⚠️ 错误"
                font.bold: true
                font.pixelSize: 18
                color: dangerColor
                Layout.alignment: Qt.AlignHCenter
            }
            Label {
                text: "筹划上限为50架!"
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
                Layout.fillWidth: true
                color: textColor
            }
            CustomButton {
                text: "确定"
                Layout.alignment: Qt.AlignHCenter
                onClicked: group_popu4.close()
                color: dangerColor
            }
        }
    }

    Popup {
        id: devide_grp_pop
        anchors.centerIn: Overlay.overlay
        width: 320
        height: 160
        modal: true
        background: Rectangle {
            color: "#3b4252"
            radius: 12
            border.color: accentColor
            border.width: 2
        }
        contentItem: ColumnLayout {
            anchors.fill: parent
            anchors.margins: 20
            spacing: 20
            Label {
                text: "⚠️ 提示"
                font.bold: true
                font.pixelSize: 18
                color: root.accentColor
                Layout.alignment: Qt.AlignHCenter
            }
            Label {
                text: "分组上限为4组，请合理设置分组!"
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
                Layout.fillWidth: true
                color: textColor
            }
            CustomButton {
                text: "确定"
                Layout.alignment: Qt.AlignHCenter
                onClicked: devide_grp_pop.close()
                color: root.primaryColor
            }
        }
    }


    property var    _sysid_list: []
    property var    idpos_map: {0:0}
    property var    hasset_map: {0:0}
    property bool ifpick: false
    property int myIntx:0
    property int myInty:0
    property int lastIntx: 0
    property int lastInty: 0
    property bool if_release: false
    property var main_node_name: []
    property var  modelmp: {0:0}
    property var form_arr: []
    property int separate_main: 1
    property int group_num: 1
    property var plan_id: []
    property var plan_arr: []
    property var select_merge: []
    property var arr_to_change_pos: []
    property var grp_pos_mp: {0:0} // 组别对应的屏幕位置
    
    // 当前选中模型相对于主机的坐标
    property int relativeEast: 0   // 东(正) 西(负)
    property int relativeNorth: 0  // 北(正) 南(负)
    property int relativeAlt: 0    // 高度差
    property string relativeMainName: ""  // 相对的主机名称
    
    // 计算相对于主机的坐标 (参考caculate_pos的计算方式)
    function updateRelativePosition(node) {
        if (!node) {
            relativeEast = 0;
            relativeNorth = 0;
            relativeAlt = 0;
            relativeMainName = "";
            return;
        }
        
        // 获取主机名称
        var mainName = main_node_name[node.group_id - 1];
        if (!mainName || mainName === "" || mainName === 0) {
            relativeEast = 0;
            relativeNorth = 0;
            relativeAlt = 0;
            relativeMainName = "";
            return;
        }
        
        relativeMainName = mainName;
        
        // 如果是主机本身，坐标为0
        if (node.objectName === mainName) {
            relativeEast = 0;
            relativeNorth = 0;
            relativeAlt = 0;
            return;
        }
        
        // 从plan_arr中查找主机节点
        var mainNode = null;
        for (var i = 0; i < plan_arr.length; i++) {
            if (plan_arr[i].objectName === mainName) {
                mainNode = plan_arr[i];
                break;
            }
        }
        
        if (!mainNode) {
            relativeEast = 0;
            relativeNorth = 0;
            relativeAlt = 0;
            return;
        }
        
        // 获取当前节点位置 - 使用model_x/y/z，这些在移动时会实时更新
        var nodeX = node.model_x || 0;
        var nodeY = node.model_y || 0;
        var nodeZ = node.model_z || 0;
        
        // 获取主机位置
        var mainX = mainNode.model_x || 0;
        var mainY = mainNode.model_y || 0;
        var mainZ = mainNode.model_z || 0;
        
        // 参考caculate_pos的计算: -(y差)*间距, (x差)*间距, -(z差)
        // 东西 = (x差) * 间距
        // 南北 = -(y差) * 间距 (屏幕Y向下为正，所以取反得到北为正)
        // 高度 = -(z差)
        var spacing = Number(input5.text) || 1;
        relativeEast = (nodeX - mainX) * spacing;
        relativeNorth = -(nodeY - mainY) * spacing;
        relativeAlt = (nodeZ - mainZ);
    }

    onUpdate_other_airplane: function(par,set,gr){ // 只在主机更新时,同组的有些模型还没加载出来,没加载出来的模型的位置会遗漏

                if (par === 2 && set === 0) { //只更新位置,所有
                    update_all_pos()
                    updateGroupCounts()  // 更新各组数量显示
                } else if (par === 2 && set === 1) {// 是最后一个且是主机
                    for (var j = 0; j < plan_arr.length; j++) {
                        if (plan_arr[j].is_connected && plan_arr[j].set_main) {
                                set_main_behavior(plan_arr[j],par)
                        }
                    }
                    updateGroupCounts()  // 更新各组数量显示
                }
                if (par === 1) {// 是主机
                    for (var h = 0; h < plan_arr.length; h++) {
                        if(plan_arr[h].is_connected && plan_arr[h].set_main) {
                            set_main_behavior(plan_arr[h],0) // 不发位置
                        }
                    }
                    updateGroupCounts()  // 更新各组数量显示
                }

    }
    function my_delay(){
        for(var i = 0;i<2000;i++){
            for(var j = 0; j <2000;j++){

            }
        }
    }

    function merge_grp(){ // 只是变更分组
        arr_to_change_pos.length = 0
        for(var i = 1; i < select_merge.length; i++) {
            for(var j = 0; j < plan_arr.length; j++) {
                if (select_merge[i].group_id === plan_arr[j].group_id && select_merge[i] !== plan_arr[j]){ // 主机改了后   后面的就无法识别了,类似引用
                   // console.log("mg",plan_arr[j].group_id,plan_arr[j].objectName)
                    plan_arr[j].group_id = select_merge[0].group_id  // 所选择主机的同组的从机
                    if(plan_arr[j].is_connected)swarm_send.store_airplane_group(plan_arr[j].objectName, plan_arr[j].group_id,true)
                    arr_to_change_pos.push(plan_arr[j])
                }
            }
            select_merge[i].group_id = select_merge[0].group_id
            arr_to_change_pos.push(select_merge[i])
            if(select_merge[i].is_connected)swarm_send.store_airplane_group(select_merge[i].objectName, select_merge[i].group_id,true)
        }
    }

    function update_all_pos(){
        console.log("update",_sysid_list.length)
        my_delay()
        my_delay()
        for(var j = 0; j <  _sysid_list.length;j++)
            swarm_send.store_airplane_group(modelmp[_sysid_list[j]].objectName, modelmp[_sysid_list[j]].group_id, true)//把已经存在的从机记录组别

        for(var i = 1; i <= main_node_name.length; i++){
            if(hasset_map[i]===1 && main_node_name[i - 1] !== 0){ // 如果这个组的主机已经加载
                my_delay()
                swarm_send.set_main_airplane(main_node_name[i - 1], i, 0, 0, 0)
                my_delay()

                send_all_airplane_pos(i,1)// 最后一个   且
            }


        }
  //  }
    }
    function send_all_airplane_pos(grp_n,send_f) { // 需要改为只发送同组的  待解决
      //

       // for (var m = 0; m < main_node_name.length; m++) { 不更新所有组了
         //   for (var i = 0;i < plan_arr.length; i++) {
         //   if (plan_arr[i].is_connected === true && main_node_name[m] === plan_arr[i].objectName && plan_arr[i].set_main === true) // 如果这个主机模型已经加载了
        for(var m = 0; m < plan_arr.length; m++) {
            if(plan_arr[m].objectName === main_node_name[grp_n - 1] && plan_arr[m].is_connected !== true){
                return // 如果主机没有连接，直接返回
            }
        }
        for (var n = 0; n < _sysid_list.length; n++) {
             //   console.log("++++++++++",_sysid_list[n],main_node_name[grp_n - 1],grp_n,idpos_map[_sysid_list[n]][0],idpos_map[_sysid_list[n]][1])
             //   if(modelmp[main_node_name[grp_n - 1]] === 0)return

                if (modelmp[_sysid_list[n]].group_id === modelmp[main_node_name[grp_n - 1]].group_id) { // 说明是同一组
                    if ( if_main_node(modelmp[_sysid_list[n]].objectName) ) {
                        swarm_send.caculate_pos(_sysid_list[n], 0, 0, 0,send_f)
                        continue;
                    }
                    if(send_f)for(var k  = 0; k < 10000000;k++){}// 避免数据拥堵
                    swarm_send.caculate_pos(_sysid_list[n],
                                              -(idpos_map[_sysid_list[n]][1] - idpos_map[main_node_name[grp_n - 1]][1]) * input5.text,
                                              (idpos_map[_sysid_list[n]][0] - idpos_map[main_node_name[grp_n - 1]][0]) * input5.text,
                                              -(idpos_map[_sysid_list[n]][2] - idpos_map[main_node_name[grp_n - 1]][2]),send_f)
                }
            }
      //  }
      //  }
    }

    function will_crush(node) {
        for(var n = 0; n < _sysid_list.length; n++)  {
            if (idpos_map[_sysid_list[n]][0] === idpos_map[node.objectName][0] &&
                    idpos_map[_sysid_list[n]][1] === idpos_map[node.objectName][1] &&
                    idpos_map[_sysid_list[n]][2] === idpos_map[node.objectName][2]
                    && Number(_sysid_list[n]) !== Number(node.objectName)) {
                return true // 检测到有
            }
        }
        return false
    }
    function set_main_color(nodeOrName) {
        // 支持传入node对象或objectName字符串
        var targetNode = null;
        var targetGroupId = 0;
        
        if (typeof nodeOrName === 'string' || typeof nodeOrName === 'number') {
            // 传入的是objectName，需要找到对应的node
            for (var i = 0; i < plan_arr.length; i++) {
                if (plan_arr[i].objectName === String(nodeOrName)) {
                    targetNode = plan_arr[i];
                    targetGroupId = plan_arr[i].group_id;
                    break;
                }
            }
        } else {
            targetNode = nodeOrName;
            targetGroupId = nodeOrName.group_id;
        }
        
        if (!targetNode || !targetGroupId) return;
        
        // 设置主机颜色，并将同组其他模型设为从机颜色
        for (var n = 0; n < plan_arr.length; n++) {
            if (plan_arr[n].group_id === targetGroupId) {
                if (if_main_node(plan_arr[n].objectName)) {
                    plan_arr[n].is_main = true;
                } else {
                    plan_arr[n].is_main = false;
                }
            }
        }
    }
    function set_main_name(node) {
        for (var n = 0; n < plan_arr.length; n++) {
            if (plan_arr[n].group_id === node.group_id) { // 只对这一组的颜色进行排他
                if (if_main_node(plan_arr[n].objectName) ) {
                    plan_arr[n].set_main = 1
                    continue;
                }
                plan_arr[n].set_main = 0
            }
        }
    }

    function trans_pos_to_grp(pos){
        for(var ke in grp_pos_mp) {
            if(grp_pos_mp[ke] === pos)
                return ke
        }
        return 0
    }
    
    // 将指定组的模型移动到指定屏幕位置，带碰撞检测
    function moveGroupToPositionWithCollision(groupId, screenPos) {
        console.log("moveGroupToPositionWithCollision: groupId=", groupId, "screenPos=", screenPos);
        
        // 根据屏幕位置计算起始坐标和边界
        // 位置: 1=左上, 2=右上, 3=左下, 4=右下
        var startX = 0;
        var startY = 0;
        var maxX = 0;
        var maxY = 0;
        var halfWidth = Math.floor(control.width / 2 / 40);
        var halfHeight = Math.floor(control.height / 2 / 40);
        var fullWidth = Math.floor(control.width / 40) - 1;
        var fullHeight = Math.floor(control.height / 40) - 1;
        
        if(screenPos === 1) {
            startX = 0; startY = 0;
            maxX = halfWidth - 1; maxY = halfHeight - 1;
        } else if(screenPos === 2) {
            startX = halfWidth + 1; startY = 0;
            maxX = fullWidth; maxY = halfHeight - 1;
        } else if(screenPos === 3) {
            startX = 0; startY = halfHeight + 1;
            maxX = halfWidth - 1; maxY = fullHeight;
        } else if(screenPos === 4) {
            startX = halfWidth + 1; startY = halfHeight + 1;
            maxX = fullWidth; maxY = fullHeight;
        }
        
        console.log("区域范围: startX=", startX, "startY=", startY, "maxX=", maxX, "maxY=", maxY);
        
        // 收集该组的所有模型
        var groupModels = [];
        for(var i = 0; i < plan_arr.length; i++) {
            if(plan_arr[i].group_id === groupId) {
                groupModels.push(plan_arr[i]);
            }
        }
        
        // 逐个放置模型，检测碰撞
        var xx = startX;
        var yy = startY;
        
        for(var j = 0; j < groupModels.length; j++) {
            var model = groupModels[j];
            
            // 找到一个没有碰撞的位置
            while(transform_crush(xx, yy, groupId) && xx <= maxX) {
                xx++;
                if(xx > maxX) {
                    xx = startX;
                    yy++;
                    if(yy > maxY) {
                        console.log("警告：区域内没有足够空间放置模型");
                        break;
                    }
                }
            }
            
            console.log("放置模型", model.objectName, "到位置:", xx, yy);
            screen_pos_to_world_pos(xx, yy, model);
            
            // 移动到下一个位置
            xx++;
            if(xx > maxX) {
                xx = startX;
                yy++;
            }
        }
    }
    
    // 直接根据组号移动该组所有模型到指定屏幕位置
    function moveGroupModels(groupId, screenPos, totalGroups) {
        var xx = 0;
        var yy = 0;
        var mstart = 0;
        var lim = 0;
        
        console.log("moveGroupModels: groupId=", groupId, "screenPos=", screenPos, "totalGroups=", totalGroups);
        
        // 根据屏幕位置和总组数计算起始坐标
        // 位置: 1=左上, 2=右上, 3=左下, 4=右下
        if(screenPos === 1) {
            xx = 0;
            yy = 0;
            mstart = 0;
            lim = Math.floor(control.width / 2 / 40) - 1;
        } else if(screenPos === 2) {
            mstart = Math.floor(control.width / 2 / 40) + 1;
            xx = mstart;
            yy = 0;
            lim = Math.floor(control.width / 40) - 2;
        } else if(screenPos === 3) {
            xx = 0;
            yy = Math.floor(control.height / 2 / 40) + 1;
            mstart = 0;
            lim = Math.floor(control.width / 2 / 40) - 1;
        } else if(screenPos === 4) {
            mstart = Math.floor(control.width / 2 / 40) + 1;
            xx = mstart;
            yy = Math.floor(control.height / 2 / 40) + 1;
            lim = Math.floor(control.width / 40) - 2;
        }
        
        console.log("位置参数: xx=", xx, "yy=", yy, "lim=", lim, "mstart=", mstart);
        
        // 直接根据group_id移动模型
        for(var i = 0; i < plan_arr.length; i++) {
            if(plan_arr[i].group_id === groupId) {
                if(xx <= lim) {
                    screen_pos_to_world_pos(xx, yy, plan_arr[i]);
                    xx++;
                } else {
                    yy++;
                    xx = mstart;
                    screen_pos_to_world_pos(xx, yy, plan_arr[i]);
                    xx++;
                }
                console.log("移动模型", plan_arr[i].objectName, "到位置:", xx-1, yy);
            }
        }
    }
    
    function move_model(n,sumn) {// n 指屏幕位置
        var xx = 0
        var yy = 0
        var mstart = 0
        var lim = 0

        console.log("move_model调用: n=", n, "sumn=", sumn);

        if (sumn === 2) {
            // 分成2组：左右分布
            if (n === 1) {
                // 第一组：左半边
                xx = 0;
                yy = 0;
                lim = Math.floor(control.width / 2 / 40) - 1; // 左半边的列数限制
            } else if (n === 2) {
                // 第二组：右半边
                mstart = Math.floor(control.width / 2 / 40) + 1; // 从中线右侧开始
                xx = mstart;
                yy = 0;
                lim = Math.floor(control.width / 40) - 2; // 右边界留空间
            }
        } else if (sumn === 3) {
            // 分成3组：上左、上右、下左
            if (n === 1) {
                // 第一组：左上角
                xx = 0;
                yy = 0;
                lim = Math.floor(control.width / 2 / 40) - 1;
            } else if (n === 2) {
                // 第二组：右上角
                mstart = Math.floor(control.width / 2 / 40) + 1;
                xx = mstart;
                yy = 0;
                lim = Math.floor(control.width / 40) - 2;
            } else if (n === 3) {
                // 第三组：左下角（靠左侧）
                xx = 0; // 从最左边开始
                yy = Math.floor(control.height / 2 / 40) + 1; // 从中线下方开始
                lim = Math.floor(control.width / 2 / 40) - 1; // 限制在左半边
                mstart = 0; // 换行时回到最左边
            }
        } else if (sumn === 4) {
            // 分成4组：四个象限
            if (n === 1) {
                // 第一组：左上角
                xx = 0;
                yy = 0;
                lim = Math.floor(control.width / 2 / 40) - 1;
            } else if (n === 2) {
                // 第二组：右上角
                mstart = Math.floor(control.width / 2 / 40) + 1;
                xx = mstart;
                yy = 0;
                lim = Math.floor(control.width / 40) - 2;
            } else if (n === 3) {
                // 第三组：左下角
                xx = 0;
                yy = Math.floor(control.height / 2 / 40) + 1;
                lim = Math.floor(control.width / 2 / 40) - 1;
            } else if (n === 4) {
                // 第四组：右下角
                mstart = Math.floor(control.width / 2 / 40) + 1;
                xx = mstart;
                yy = Math.floor(control.height / 2 / 40) + 1;
                lim = Math.floor(control.width / 40) - 2;
            }
        }

        console.log("位置参数: xx=", xx, "yy=", yy, "lim=", lim, "mstart=", mstart);

        for (var i = 0; i < plan_arr.length; i++) {
            if (plan_arr[i].group_id === Number(trans_pos_to_grp(n))) {
                if(xx < lim) {
                    screen_pos_to_world_pos(xx, yy, plan_arr[i]);
                    xx++;
                } else {
                    yy++;
                    xx = mstart;
                    screen_pos_to_world_pos(xx, yy, plan_arr[i]);
                    xx++;
                }
                console.log("放置模型", plan_arr[i].objectName, "在位置:", xx-1, yy);
            }
        }
    }

    function set_main_behavior(node,send) { //

        if(!send) {
            my_delay()
            my_delay()
        }
        swarm_send.set_main_airplane(main_node_name[node.group_id - 1], node.group_id,
                                     0,
                                     0,
                                     0)
        //swarm_send.caculate_pos(node.objectName, 0, 0, 0)

        set_main_color(node)
        hasset_map[node.group_id]=1

        if(send !== 0) { // 失去继承设置主机的意义了?
           // send_all_airplane_pos(node.group_id,1) //0 :不延时
            update_all_pos()
        }
        console.log("set_main_behavior ",main_node_name[node.group_id - 1],node.group_id,send)
    }

    function show_position(node){
        var map_from_1 = control.mapFrom3DScene(node.scenePosition) // 屏幕坐标

        myIntx = (map_from_1.x - 20) % 40 > 25 ? (map_from_1.x - 20) / 40 + 1 : (map_from_1.x - 20) / 40
        myInty = (map_from_1.y - 20) % 40 > 25 ? (map_from_1.y - 20) / 40 + 1 : (map_from_1.y - 20) / 40

        node.model_x = myIntx
        node.model_y = myInty
    }
    function screen_pos_to_world_pos(xx,yy,thisnode){ // 间距
        thisnode.visible = true
        xx = xx * 40 + 20  // 像素中心
        yy = yy * 40 + 20
        var map_from_1 = control.mapFrom3DScene(thisnode.scenePosition) // 屏幕坐标

        var x_off = 0
        var y_off = 0
        var pos_temp = 0
        var map_to = 0
        while(Math.abs(xx - map_from_1.x) > 1 || Math.abs(yy - map_from_1.y) > 1) {
            x_off = xx - map_from_1.x
            y_off = yy - map_from_1.y
        //    console.log(x_off,y_off,map_from_1.x,map_from_1.y)
            pos_temp = Qt.vector3d((map_from_1.x + x_off), (map_from_1.y + y_off), map_from_1.z);
            map_to = control.mapTo3DScene(pos_temp) // 世界坐标
            thisnode.x = map_to.x
            thisnode.y = map_to.y
            map_from_1 = control.mapFrom3DScene(thisnode.scenePosition) // 屏幕坐标
        }
        myIntx = (map_from_1.x - 20) % 40 > 25 ? (map_from_1.x - 20) / 40 + 1 : (map_from_1.x - 20) / 40
        myInty = (map_from_1.y - 20) % 40 > 25 ? (map_from_1.y - 20) / 40 + 1 : (map_from_1.y - 20) / 40
        thisnode.model_x = myIntx
        thisnode.model_y = myInty
        if (thisnode.is_connected) {
            idpos_map[Number(thisnode.objectName)][0] = myIntx
            idpos_map[Number(thisnode.objectName)][1] = myInty
        }
    }

    function display_changed_pos(grp_id){
        var i = 0;
        for(;i<plan_arr.length;i++) {
            if(plan_arr[i].group_id === grp_id && plan_arr[i].set_main)break
        }
        var x_0 = plan_arr[i].is_connected? idpos_map[plan_arr[i].objectName][0]:plan_arr[i].model_x // 是主机
        var y_0 = plan_arr[i].is_connected? idpos_map[plan_arr[i].objectName][1]:plan_arr[i].model_y
        x_0++

        var x_lim = (root.width -20 ) / 40
        var y_lim = (root.height -20 ) / 40
        while(transform_crush(x_0,y_0,mouse_area.pickNode.group_id)) {
           // if() 需判断临界时换行
            x_0++
        }
        screen_pos_to_world_pos(x_0,y_0,mouse_area.pickNode)
    }
    function show_line(n){ // 函数是否还有调用
        for(var i = 0; i < plan_arr.length;i++) {
            if(plan_arr[i].group_id === n){
                return true
            }
        }
        return false
    }
    // 什么时候用：在独立分组时，如果此分组的所有成员都在本区域，则返回真
    // 用来干什么
    function judge_this_area() {
        for(var i = 0; i < plan_arr.length; i++) {
            var x_0 = plan_arr[i].is_connected? idpos_map[plan_arr[i].objectName][0]:plan_arr[i].model_x // 是主机
            var y_0 = plan_arr[i].is_connected? idpos_map[plan_arr[i].objectName][1]:plan_arr[i].model_y
            if(grp_pos_mp[plan_arr[i].group_id] === 1 && (x_0 >= 9)) {

            }
        }
    }
    function all_move_by_line() {
        var max_y = 0
        var min_y = 18
        var max_x = 0
        var min_x = 34
        var index = 0
        var towards = 0
        for (var i = 0; i < plan_arr.length; i++) {
            var x_0 = plan_arr[i].is_connected? idpos_map[plan_arr[i].objectName][0]:plan_arr[i].model_x // 是主机
            var y_0 = plan_arr[i].is_connected? idpos_map[plan_arr[i].objectName][1]:plan_arr[i].model_y

            if (canv3.visible && y_0 >= 9 && grp_pos_mp[plan_arr[i].group_id] === 1){
                if(max_y < y_0) {
                    max_y = y_0
                    index = i
                    towards = 1
                }
            }
            if(canv4.visible && y_0 >= 9 && grp_pos_mp[plan_arr[i].group_id] === 2) {
                if(max_y < y_0) {
                    max_y = y_0
                    index = i
                    towards = 1
                }
            }
            if(canv3.visible && y_0 <= 9 && grp_pos_mp[plan_arr[i].group_id] === 3) {
                if(min_y > y_0) {
                    min_y = y_0
                    index = i
                    towards = 2
                }
            }
            if(canv4.visible && y_0 <= 9 && grp_pos_mp[plan_arr[i].group_id] === 4){
                if(min_y > y_0) {
                    min_y = y_0
                    index = i
                    towards = 2
                }
            }
            if ((canv.visible && x_0 >= 18 && grp_pos_mp[plan_arr[i].group_id] === 1) || (canv2.visible && x_0 >= 18 && grp_pos_mp[plan_arr[i].group_id] === 3)){ // 给左
                if(max_x < x_0) {//...
                    max_x = x_0
                    index = i
                    towards = 3
                }
            }
            if ((canv.visible && x_0 <= 18 && grp_pos_mp[plan_arr[i].group_id] === 2) || (canv2.visible && x_0 <= 18 && grp_pos_mp[plan_arr[i].group_id] === 4)){
                if(min_x > x_0) { // 给右
                    min_x = x_0
                    index = i
                    towards = 4
                }
            }

        }

        for(var j = 0; j < plan_arr.length;j++) {
            if(plan_arr[j].group_id === plan_arr[index].group_id){
                x_0 = plan_arr[j].is_connected? idpos_map[plan_arr[j].objectName][0]:plan_arr[j].model_x
                y_0 = plan_arr[j].is_connected? idpos_map[plan_arr[j].objectName][1]:plan_arr[j].model_y
                if(towards === 1)
                    screen_pos_to_world_pos(x_0, y_0 - (max_y - 9) - 1,plan_arr[j])
                else if (towards === 2)
                    screen_pos_to_world_pos(x_0, y_0 + min_y,plan_arr[j])
                else if (towards === 3)
                    screen_pos_to_world_pos(x_0 - (max_x - 17) - 1, y_0,plan_arr[j])
                else if (towards === 4)
                    screen_pos_to_world_pos(x_0 + min_x, y_0,plan_arr[j])// +的有点多
            }
        }
    }

    function grp_has_pos(pos) {
        for(var ke in grp_pos_mp){
         //   console.log("pos",ke,grp_pos_mp[ke])
            if(grp_pos_mp[ke] === pos)
                return true
        }
        return false
    }

    function devide_screen(grp){//仅在独立分组时用
        if(group_num === 2){
           // if(main_node_name)
            if((grp_has_pos(1) && grp_has_pos(2)) || (grp_has_pos(1) && grp_has_pos(4)) || (grp_has_pos(3) && grp_has_pos(2)) || (grp_has_pos(3) && grp_has_pos(4))) {
                canv.visible = true
                canv2.visible = true
                canv3.visible = false
                canv4.visible = false

                move_model(2,2)
            }
         /*   else if ((grp_has_pos(1) && grp_has_pos(3)) || (grp_has_pos(2) && grp_has_pos(4))) {
                canv.visible = false
                canv2.visible = false
                canv3.visible = true
                canv4.visible = true
            }*/
         /*   if(grp === 1)
                move_model(1,2)*/
          //  move_model(2,3)

        }

      /*  if(group_num === 3){

            canv.visible = true
            canv4.visible = true
            canv2.visible = true
            canv3.visible = true

            move_model(3,3)
        }*/
        if(group_num === 4 || group_num === 3){
            canv.visible = true
            canv4.visible = true
            canv2.visible = true
            canv3.visible = true
            if(grp_has_pos(1) === false) {
                grp_pos_mp[grp] = 1
                move_model(1, group_num)
            } else if (grp_has_pos(2) === false) {
                grp_pos_mp[grp] = 2
                move_model(2, group_num)
            } else if (grp_has_pos(3) === false) {
                grp_pos_mp[grp] = 3
                move_model(3, group_num)
            } else if (grp_has_pos(4) === false) {
                grp_pos_mp[grp] = 4
                move_model(4, group_num)
            }
         /*   if(grp === 1)
                move_model(1,2)
            else if (grp === 2)
                move_model(2,2)
            else if (grp === 3)
                move_model(3,4)
            else if (grp === 4)
                move_model(4,4)
            */
        }
    }

    function find_max_pos(grp, move_grp){ // 传入待合入的组别
        var max_x = 0
        var max_y = 0
        var towards = 0

        var merge_limx = 0
        var merge_limy = 0
        for(var j = 0; j < plan_arr.length; j++) {
            if(plan_arr[j].group_id === grp) {
                max_x = plan_arr[j].is_connected ? idpos_map[plan_arr[j].objectName][0]:plan_arr[j].model_x
                max_y = plan_arr[j].is_connected ? idpos_map[plan_arr[j].objectName][1]:plan_arr[j].model_y
                break
            }
        }
        for(var j2 = 0; j2 < plan_arr.length; j2++) {
            if(plan_arr[j2].group_id === move_grp) {
                merge_limx = plan_arr[j2].is_connected ? idpos_map[plan_arr[j2].objectName][0]:plan_arr[j2].model_x
                merge_limy = plan_arr[j2].is_connected ? idpos_map[plan_arr[j2].objectName][1]:plan_arr[j2].model_y
                break
            }
        }
      //  console.log("find_max grp  main:max_x max_y",grp,move_grp,max_x,max_y)
        var xx = 0
        var yy = 0
        for(var i = 0; i < plan_arr.length; i++) {

            if(plan_arr[i].group_id === grp) { // grp就是主机的组别
                xx = plan_arr[i].is_connected ? idpos_map[plan_arr[i].objectName][0]:plan_arr[i].model_x  // model_x  全是0？
                yy = plan_arr[i].is_connected ? idpos_map[plan_arr[i].objectName][1]:plan_arr[i].model_y
                 if (grp_pos_mp[grp] ===  1 && grp_pos_mp[move_grp] === 2 ){ // 给右
                    if(max_x < xx) {max_x = xx;max_y = yy}
                    towards = 4
                } else if ((grp_pos_mp[grp] ===  1 && grp_pos_mp[move_grp] === 3)) { // 给下排
                    if(max_y < yy) max_y = yy
                    towards = 2
                }
/*
                if (grp_pos_mp[grp] ===  1 && grp_pos_mp[move_grp] === 4) {//左上
                    if(max_y < yy) max_y = yy
                    if(max_x < xx) max_x = xx
                    towards = 5
                }
*/
                if(grp_pos_mp[grp] ===  2 && grp_pos_mp[move_grp] === 1) { // 给左
                    if(max_x > xx) max_x = xx
                   // if(max_y < y) max_y = y
                    towards = 3
                } else if((grp_pos_mp[grp] ===  2 && grp_pos_mp[move_grp] === 3)||(grp_pos_mp[grp] ===  2 && grp_pos_mp[move_grp] === 4)) { // 给下
                    if(max_y < yy) max_y = yy
                    towards = 2
                }

                if(grp_pos_mp[grp] ===  3 && grp_pos_mp[move_grp] === 4) {// 给右面
                    if(max_x < xx) max_x = xx
                    towards = 4
                } else if ((grp_pos_mp[grp] ===  3 && grp_pos_mp[move_grp] === 1) || (grp_pos_mp[grp] ===  3 && grp_pos_mp[move_grp] === 2)){ // 给上
                    if(max_y > yy )max_y = yy
                    towards = 1
                }
                if(grp_pos_mp[grp] ===  4 && grp_pos_mp[move_grp] === 2) { // 给上
                    if(max_y > yy) max_y = yy
                    towards = 1
                } else if ((grp_pos_mp[grp] ===  4 && grp_pos_mp[move_grp] === 1) || (grp_pos_mp[grp] ===  4 && grp_pos_mp[move_grp] === 3)){// 给左
                    if(max_x > xx) max_x = xx
                    towards = 3
                }
            }

        }
        for(i = 0; i < plan_arr.length; i++) {
            if(plan_arr[i].group_id === move_grp){
                xx = plan_arr[i].is_connected ? idpos_map[plan_arr[i].objectName][0]:plan_arr[i].model_x
                yy = plan_arr[i].is_connected ? idpos_map[plan_arr[i].objectName][1]:plan_arr[i].model_y
                if (towards === 1) {
                    if(merge_limy < yy)merge_limy = yy
                }
                if (towards === 2) {
                    if(merge_limy > yy)merge_limy = yy
                }
                if (towards === 3) {
                    if(merge_limx < xx)merge_limx = xx
                }
                if (towards === 4) {
                    if(merge_limx > xx)merge_limx = xx
                }
            }
        }

      //  console.log("find mgrp x  y  t",grp,max_x,max_y,merge_limx,merge_limy,towards)
        wait_to_merge_pos(max_x,max_y,merge_limx,merge_limy,towards,grp,move_grp)
    }
    // 在更改分组前移动  或者
    function wait_to_merge_pos(max_x,max_y,merge_limx,merge_limy,towards,main_grp,move_grp){
        if((grp_pos_mp[main_grp] === 1 && 3 === grp_pos_mp[move_grp]) || (grp_pos_mp[main_grp] === 3 && 1 === grp_pos_mp[move_grp])) {
            canv3.visible = false
        }
        if((grp_pos_mp[main_grp] === 1 && 2 === grp_pos_mp[move_grp]) || (grp_pos_mp[main_grp] === 2 && 1 === grp_pos_mp[move_grp])) {
            canv.visible = false
        }
        if((grp_pos_mp[main_grp] === 4 && 3 === grp_pos_mp[move_grp]) || (grp_pos_mp[main_grp] === 3 && 4 === grp_pos_mp[move_grp])) {
            canv2.visible = false
        }
        if((grp_pos_mp[main_grp] === 2 && 4 === grp_pos_mp[move_grp]) || (grp_pos_mp[main_grp] === 4 && 2 === grp_pos_mp[move_grp])) {
            canv4.visible = false
        }
        if((grp_pos_mp[main_grp] === 1 && 4 === grp_pos_mp[move_grp]) || (grp_pos_mp[main_grp] === 4 && 1 === grp_pos_mp[move_grp])) {
            canv4.visible = false
        }
        if(grp_pos_mp[main_grp] === 2 && 3 === grp_pos_mp[move_grp]) {
           // canv4.visible = false
        }
        var x_0 = 0
        var y_0 = 0
        var half_width = (root.width - 20) / 80
        var half_height = (root.height - 20) / 80
        for (var i = 0; i < plan_arr.length; i++) {
            if(plan_arr[i].group_id === move_grp) { // move 第一组
                x_0 = plan_arr[i].is_connected ? _sysid_list[plan_arr[i].objectName][0] :plan_arr[i].model_x
                y_0 = plan_arr[i].is_connected ? _sysid_list[plan_arr[i].objectName][1] :plan_arr[i].model_y
                if(towards === 1) { // 上方
                  /*  while(transform_crush(x_0,y_0,mouse_area.pickNode.group_id)) {
                       // if() 需判断临界时换行
                        x_0++
                    }*/
                    if ((grp_pos_mp[plan_arr[i].group_id] === 1 && grp_pos_mp[main_grp] === 3) ||
                            (grp_pos_mp[plan_arr[i].group_id] === 2 && grp_pos_mp[main_grp] === 4)) {
                        screen_pos_to_world_pos(x_0, y_0 - (merge_limy - max_y) - 1,plan_arr[i]) //y + offset, offset = max_y - y
                    }
                }
                if(towards === 2) { // 下方
                    if ((grp_pos_mp[plan_arr[i].group_id] === 3 && grp_pos_mp[main_grp] === 1) ||
                            (grp_pos_mp[plan_arr[i].group_id] === 4 && grp_pos_mp[main_grp] === 2)) {
                        screen_pos_to_world_pos(x_0,y_0 + (max_y - merge_limy) + 1,plan_arr[i]) //y + offset, offset = max_y - y
                    }
                }
                if(towards === 3) { // 左方
                    if ((grp_pos_mp[plan_arr[i].group_id] === 1 && grp_pos_mp[main_grp] === 2) ||
                            (grp_pos_mp[plan_arr[i].group_id] === 3 && grp_pos_mp[main_grp] === 4)) {
                        screen_pos_to_world_pos(x_0 + (max_x - merge_limx) - 1,y_0,plan_arr[i]) //y + offset, offset = max_y - y
                    }
                }
                if(towards === 4) { // 右方
                    if ((grp_pos_mp[plan_arr[i].group_id] === 2 && grp_pos_mp[main_grp] === 1) ||
                            (grp_pos_mp[plan_arr[i].group_id] === 4 && grp_pos_mp[main_grp] === 3)) {
                        screen_pos_to_world_pos(x_0 - (merge_limx - max_x) + 1,y_0,plan_arr[i]) //y + offset, offset = max_y - y
                    }
                }
            }
        }
    }

    function if_main_node(objname) {
        for(var i = 0; i < main_node_name.length; i++) {
            if (main_node_name[i] === objname) {
                return true
            }
        }
        return false
    }
    function reset_main_name(old,newname){
        for(var i = 0; i < main_node_name.length; i++) {
            if (main_node_name[i] === old) {
                main_node_name[i] = newname
                return
            }
        }
    }
    function get_pos(node) {
        var map_from_1 = control.mapFrom3DScene(node.scenePosition) // 屏幕坐标
        myIntx = (map_from_1.x - 20) % 40 > 25 ? (map_from_1.x - 20) / 40 + 1 : (map_from_1.x - 20) / 40
        myInty = (map_from_1.y - 20) % 40 > 25 ? (map_from_1.y - 20) / 40 + 1 : (map_from_1.y - 20) / 40
        node.model_x = myIntx
        node.model_y = myInty
    }
    function plan_to_visible() {
       // console.log("sss",plan_id.length,input_plan.text)
        for (var i = 0; i < plan_id.length; i++) {
            plan_id[i].visible = false
            plan_id[i].pickable = false
        }
        plan_arr.length = 0
        my_delay()
        for (i = 0; i < Number(input_plan.text); i++) {
            plan_id[i].visible = true
            plan_id[i].pickable = true
            plan_arr.push(plan_id[i])
           // console.log(plan_id[i].objectName,plan_id[i].visible,plan_id[i].pickable)
        }
    }

    function plan_to_out_main(node) {
        for (var i = 0; i < plan_id.length; i++) {
            if (node.group_id === plan_id[i].group_id && node.objectName !== plan_id[i].objectName && plan_id[i].visible === true) {
                plan_id[i].set_main = 0
            }
        }
    }
    function mymove(xx,yy,thisnode){
        // 检查View3D尺寸是否有效
     /*   if (control.width <= 0 || control.height <= 0) {
            console.log("警告：View3D尺寸无效，延迟进行坐标转换");
            // 设置一个定时器，稍后再尝试
            Qt.callLater(function() {
                if (control.width > 0 && control.height > 0) {
                    // View3D已经初始化，重新执行坐标转换
                    performMove(xx, yy, thisnode);
                } else {
                    console.log("View3D仍然未正确初始化");
                }
            });
            return;
        }*/

        performMove(xx, yy, thisnode);
    }

    function performMove(xx, yy, thisnode) {
        // 实际执行移动的函数
        thisnode.visible = false
        plan_id.push(thisnode)
/*
        var map_from_1 = control.mapFrom3DScene(node.scenePosition) // 屏幕坐标

        if (((map_from_1.x -20) % 40 <= 1 || (map_from_1.x -20) % 40 >= 39) && ((map_from_1.y-20) % 40 <= 1 || (map_from_1.y-20) % 40 >= 39)) {
            return
        }
        var nu_x = (map_from_1.x-20) % 40 > 20 ? map_from_1.x + 40 - (map_from_1.x-20) % 40 : map_from_1.x - (map_from_1.x-20) % 40;
        var nu_y = (map_from_1.y-20) % 40 > 20 ? map_from_1.y + 40 - (map_from_1.y-20) % 40 : map_from_1.y - (map_from_1.y-20) % 40;

        var pos_temp_1 = Qt.vector3d(nu_x,nu_y, map_from_1.z);
        var map_to_1 = control.mapTo3DScene(pos_temp_1) // 世界坐标
        node.x = map_to_1.x
        node.y = map_to_1.y
*/
        xx = xx * 40 + 20  // 像素中心
        yy = yy * 40 + 20

        // 1. 打印View3D的尺寸（确认屏幕坐标范围）
      //  console.log("View3D尺寸：宽=", control.width, "高=", control.height);
        // 2. 打印目标屏幕坐标（判断是否超出View3D范围）
      //  console.log("目标屏幕坐标：xx=", xx, "yy=", yy);

        var map_from_1 = control.mapFrom3DScene(thisnode.scenePosition) // 屏幕坐标
      //  console.log("初始屏幕坐标：x=", map_from_1.x, "y=", map_from_1.y, "z=", map_from_1.z);

        // 确保屏幕坐标有效
        if (isNaN(map_from_1.x) || isNaN(map_from_1.y)) {
           // console.log("警告：无效的屏幕坐标，跳过移动");
            return;
        }

        var x_off = 0
        var y_off = 0
        var pos_temp = 0
        var map_to = 0
        var maxIterations = 100; // 添加最大迭代次数限制，避免无限循环
        var iterations = 0;

        while((Math.abs(xx - map_from_1.x) > 1 || Math.abs(yy - map_from_1.y) > 1) && iterations < maxIterations) {
            x_off = xx - map_from_1.x
            y_off = yy - map_from_1.y
            //    console.log(x_off,y_off,map_from_1.x,map_from_1.y)
            pos_temp = Qt.vector3d((map_from_1.x + x_off), (map_from_1.y + y_off), map_from_1.z);
            map_to = control.mapTo3DScene(pos_temp) // 世界坐标

            // 检查世界坐标是否有效
            if (!isNaN(map_to.x) && !isNaN(map_to.y)) {
                thisnode.x = map_to.x
                thisnode.y = map_to.y
            }

            map_from_1 = control.mapFrom3DScene(thisnode.scenePosition) // 屏幕坐标
            iterations++;
        }

        if (iterations >= maxIterations) {
           // console.log("警告：达到最大迭代次数，可能未完成精确移动");
        }

        // 只有在坐标有效时才更新model_x和model_y
        if (!isNaN(map_from_1.x) && !isNaN(map_from_1.y)) {
            myIntx = (map_from_1.x - 20) % 40 > 25 ? (map_from_1.x - 20) / 40 + 1 : (map_from_1.x - 20) / 40
            myInty = (map_from_1.y - 20) % 40 > 25 ? (map_from_1.y - 20) / 40 + 1 : (map_from_1.y - 20) / 40
            thisnode.model_x = myIntx
            thisnode.model_y = myInty
           // console.log(thisnode.model_x,thisnode.model_y,thisnode.visible,map_from_1.x,map_from_1.y)
          //  console.log("最终3D坐标：x=", thisnode.x, "y=", thisnode.y, "z=", thisnode.z);
          //  console.log("相机位置：", control.camera.position, "近裁剪面：", control.camera.near, "远裁剪面：", control.camera.far);
        }
        // 这些操作已经在performMove函数中完成
        // 如果performMove函数没有被调用（例如View3D尺寸无效），则不执行这些操作
    /*
        var map_from = control.mapFrom3DScene(node.scenePosition) // 屏幕坐标

        var pos_temp = Qt.vector3d((map_from.x + n), map_from.y, map_from.z);
        var map_to = control.mapTo3DScene(pos_temp) // 世界坐标
        sphere_node.x = map_to.x
        console.log("index ",mygrid.indexAt(2,70))*/


      //  numberDelegate.itemAt(530,30).Text="1"
     //   console.log(n,map_from.x,sphere_node.x,node.scenePosition.x,"+++++++++++++++++++++++++++++++++")
    }
   /* function show_position(node){
        var map_from_1 = control.mapFrom3DScene(node.scenePosition) // 屏幕坐标

        myIntx = (map_from_1.x - 20) % 40 > 25 ? (map_from_1.x - 20) / 40 + 1 : (map_from_1.x - 20) / 40
        myInty = (map_from_1.y - 20) % 40 > 25 ? (map_from_1.y - 20) / 40 + 1 : (map_from_1.y - 20) / 40

        node.model_x = myIntx
        node.model_y = myInty
    }
    function screen_pos_to_world_pos(xx,yy,thisnode){ // 间距
        thisnode.visible = true
        xx = xx * 40 + 20  // 像素中心
        yy = yy * 40 + 20
        var map_from_1 = control.mapFrom3DScene(thisnode.scenePosition) // 屏幕坐标

        var x_off = 0
        var y_off = 0
        var pos_temp = 0
        var map_to = 0
        while(Math.abs(xx - map_from_1.x) > 1 || Math.abs(yy - map_from_1.y) > 1) {
            x_off = xx - map_from_1.x
            y_off = yy - map_from_1.y
        //    console.log(x_off,y_off,map_from_1.x,map_from_1.y)
            pos_temp = Qt.vector3d((map_from_1.x + x_off), (map_from_1.y + y_off), map_from_1.z);
            map_to = control.mapTo3DScene(pos_temp) // 世界坐标
            thisnode.x = map_to.x
            thisnode.y = map_to.y
            map_from_1 = control.mapFrom3DScene(thisnode.scenePosition) // 屏幕坐标
        }
        myIntx = (map_from_1.x - 20) % 40 > 25 ? (map_from_1.x - 20) / 40 + 1 : (map_from_1.x - 20) / 40
        myInty = (map_from_1.y - 20) % 40 > 25 ? (map_from_1.y - 20) / 40 + 1 : (map_from_1.y - 20) / 40
        thisnode.model_x = myIntx
        thisnode.model_y = myInty
        if (thisnode.is_connected) {
            idpos_map[Number(thisnode.objectName)][0] = myIntx
            idpos_map[Number(thisnode.objectName)][1] = myInty
        }
    }*/

    function transform_crush(a,b,grp_id) {
        for (var i = 0; i < _sysid_list.length; i++) {
            if (idpos_map[_sysid_list[i]][0] === a &&
                    idpos_map[_sysid_list[i]][1] === b) {
               // console.log("crash",_sysid_list[n],node.objectName,idpos_map[node.objectName])
                return true // 检测到有
            }
        }
        for(var j = 0; j < plan_arr.length;j++) {
            if(plan_arr[j].is_connected === false && plan_arr[j].model_x === a && plan_arr[j].model_y === b)return true
        }
        return false
    }
    // 三角形
    function triangle_swarm()
    {
        form_arr.length = 0
        for(var i = 0; i < plan_arr.length; i++) {
            if (plan_arr[i].group_id === Number(input4.text)) {
                form_arr.push(plan_arr[i])
            }
        }
    /*    if (form_arr.length !== 3) {
            console.log("本组飞机数量 airplane count :",form_arr.length)
            return
        }*/
        var x_0 = 0
        var y_0 = 2
        if (Number(input4.text) === 1) { // 横坐标起始点（基准线）
            x_0 = 4
        } else if(Number(input4.text) === 2) {
            x_0 = 20
        } else if(Number(input4.text) === 3) {
            x_0 = 4
            y_0 = 10
        } else if(Number(input4.text) === 4) {
            x_0 = 20
            y_0 = 10
        }

     /*   while (transform_crush(x_0,y_0,form_arr[0].group_id) || transform_crush(x_0 - 2,y_0 + 3,form_arr[1].group_id) ||
               transform_crush(x_0 + 2,y_0 + 3,form_arr[2].group_id)) {
            x_0 += 1
            y_0 += 1
        }*/

        var n = form_arr.length;
        var rows = 0;

        // 计算行数
        while (n > 0) {
            rows++; // rows 有bug，11 12 13 14 15时不准确，
            n= n-(rows*2-1)
        }

        n = form_arr.length;
        var index = 0;
        var line = 0
        for (i = 1; i <= rows; ++i) {
            var x1 = x_0
            for (var j = 0; j < rows - i; ++j) {
                x1++
            }

            line++
            for (j = 0; j < 2 * i - 1; ++j) {
                if (index < n) {
                    if(line !== rows) {
                        screen_pos_to_world_pos(x1 + j, y_0, form_arr[index++])
                      //  console.log("l,r",line,rows)
                    } else {// 对最后一行的特殊处理
                        var last_line_num = form_arr.length - index
                        screen_pos_to_world_pos(x1 + j, y_0, form_arr[index++])

                      //  console.log("最后一排的数量,两顶点位置",last_line_num,x1,x1 + 2*(i-1)) // 1、找两定点，2、计算定点间的距离，3、按数量平分
                        if (last_line_num !== 1) {// x1是第一顶点，x1 + 2*i-1
                                var ste = 2 * (i - 1) / (last_line_num - 1)
                                var step =  Math.floor(ste) // 取最大整数
                             //   console.log(ste,step)
                                for(var k = 1; k < last_line_num;k++) {
                                    if(index !== form_arr.length - 1)
                                        screen_pos_to_world_pos(x1 + k * step, y_0, form_arr[index++])
                                    else
                                       screen_pos_to_world_pos(x1 + 2*(i-1), y_0, form_arr[index++])
                                }
                                break
                        }
                    }
                }
            }

            // 换行
            y_0++
        }


        /*
        screen_pos_to_world_pos(x_0, y_0, form_arr[0]) // 里面会设置可见
        screen_pos_to_world_pos(x_0 - 2, y_0 + 3,form_arr[1])
        screen_pos_to_world_pos(x_0 + 2, y_0 + 3,form_arr[2])*/
        send_all_airplane_pos(input4.text,0)

    }
    //正方形
    function rectangle_swarm()
    {

        form_arr.length = 0
        for(var i = 0; i < plan_arr.length; i++) {
            if (plan_arr[i].group_id === Number(input4.text)) {
                form_arr.push(plan_arr[i])
            }
        }
        if (form_arr.length < 4) {
            console.log("本组飞机数量 airplane count :",form_arr.length)
            return
        }
        var x_0 = 0
        var y_0 = 2
        if (Number(input4.text) === 1) { // 横坐标起始点（基准线）
            x_0 = 2
        } else if(Number(input4.text) === 2) {
            x_0 = 20
        } else if(Number(input4.text) === 3) {
            x_0 = 2
            y_0 = 10
        } else if(Number(input4.text) === 4) {
            x_0 = 20
            y_0 = 10
        }

        var n = form_arr.length;

            // 计算正方形的边长
            var side = Math.ceil(Math.sqrt(n));
            if (side < 2) side = 2; // 最小边长为2
            var index = 0;

            // 填充四个顶点

            index += 4;

            screen_pos_to_world_pos(x_0,y_0,form_arr[0])
            screen_pos_to_world_pos(x_0,y_0 + side - 1,form_arr[1])
            screen_pos_to_world_pos(x_0 + side - 1,y_0,form_arr[2])
            screen_pos_to_world_pos(x_0 + side - 1,y_0 + side - 1,form_arr[3])

            // 填充边
            // 上边（从左到右）
            for ( i = 1; i < side - 1; ++i) {
                if (index >= n) break;
                screen_pos_to_world_pos(x_0 + i,y_0,form_arr[index])
                index++;
            }

            // 右边（从上到下）
            for ( i = 1; i < side - 1; ++i) {
                if (index >= n) break;
                screen_pos_to_world_pos(x_0 + side - 1,y_0 + i,form_arr[index])
                index++;
            }

            // 下边（从右到左）
              for (i = side - 2; i >= 1; --i) {
                  if (index >= n) break;
                  screen_pos_to_world_pos(x_0 + i,y_0 + side - 1,form_arr[index])
                  index++;
              }

            // 左边（从下到上）
            for (i = side - 2; i >= 1; --i) {
                if (index >= n) break;
                screen_pos_to_world_pos(x_0,y_0 + i,form_arr[index])
                index++;
            }

            // 填充内部
            for (i = 1; i < side - 1; ++i) {
                for (var j = 1; j < side - 1; ++j) {
                    if (index >= n) break;
                    screen_pos_to_world_pos(x_0+j,y_0 + i,form_arr[index])
                    index++;
                }
            }

        /*
        screen_pos_to_world_pos(x_0,y_0,form_arr[0])
        screen_pos_to_world_pos(x_0,y_0 + 3,form_arr[1])
        screen_pos_to_world_pos(x_0 + 3,y_0,form_arr[2])
        screen_pos_to_world_pos(x_0 + 3,y_0 + 3,form_arr[3])*/

        send_all_airplane_pos(input4.text,0)
    }
    // 菱形
    function diamond_swarm() {
        form_arr.length = 0
        for(var i = 0; i < plan_arr.length; i++) {
            if (plan_arr[i].group_id === Number(input4.text)) {
                form_arr.push(plan_arr[i])
            }
        }
        if (form_arr.length < 4) {
            console.log("本组飞机数量 airplane count :",form_arr.length)
            return
        }
        var x_0 = 0
        var y_0 = 1
        if (Number(input4.text) === 1) { // 横坐标起始点（基准线）
            x_0 = 3
        } else if(Number(input4.text) === 2) {
            x_0 = 20
        } else if(Number(input4.text) === 3) {
            x_0 = 3
            y_0 = 10
        } else if(Number(input4.text) === 4) {
            x_0 = 20
            y_0 = 10
        }
/*
        if (_sysid_list.length != 4) {
            console.log("airplane count :",_sysid_list.length)
            return
        }*/
/*
        while (transform_crush(x_0,y_0,form_arr[0].group_id) || transform_crush(x_0 - 1,y_0 + 2,form_arr[1].group_id) ||
               transform_crush(x_0 + 1,y_0 + 2,form_arr[2].group_id) || transform_crush(x_0,y_0 + 4,form_arr[3].group_id)) {
            x_0 += 1
            y_0 += 1
        }
*/

            var n = form_arr.length
            // 计算菱形的高度
            var height = 2 // 最小高度为2（4个顶点）
            while (4 + 4 * (height - 2) < n) {
                height++
            }
         //   height = 1
         //   while((2 * height - 1) * (2 * height - 1) / 2 < n) {
         //       height++
         //   }
            var size = 2 * height - 1 // 菱形的总行数
          //  height++

       // var size = 3
      //  while(size* size / 2 <= n) {
      //                  size++
      //              }
      //  size = Math.floor(Math.ceil(Math.sqrt(2*n)))
     //   if (size < 2) size = 2; // 最小边长为2
     //   height = Math.floor((size + 1) / 2)
     //   size = height * 2 - 1

           // var size = Math.floor(Math.floor(Math.sqrt(2*n)))//根号2倍的side
          //  size = size % 2 === 0 ? size + 1: size // size 变为奇数
          //  height = ((size+1)/2)


      //  var side = (Math.sqrt(n)) * Math.sqrt(2) / 2
      //  height = Math.floor(side) + 1
      //  size = 2 * height - 1
            var index = 0;

            // 填充四个顶点

            //console.log(size, height, x_0,y_0)
            index += 4
            screen_pos_to_world_pos(x_0 + height - 1,y_0 ,form_arr[0])
            screen_pos_to_world_pos(x_0,y_0 + height - 1,form_arr[1])
            screen_pos_to_world_pos(x_0 + size - 1,y_0 + height - 1,form_arr[2])
            screen_pos_to_world_pos(x_0 + height - 1,y_0 + size - 1,form_arr[3])

            // 填充边
            // 上部分（从上到下）
            for ( i = 1; i < height - 1; ++i) {
                if (index >= n) break;
                screen_pos_to_world_pos(x_0 + height - 1 - i,y_0 + i,form_arr[index])  // 左上边
                index++;
                if (index >= n) break;
                screen_pos_to_world_pos(x_0 + height - 1 + i,y_0 + i,form_arr[index]) // 右上边
                index++;
            }

            // 下部分（从上到下）
            for ( i = 1; i < height - 1; ++i) {
                if (index >= n) break;
                screen_pos_to_world_pos(x_0 + height - 1 - i,y_0 + size - 1 - i,form_arr[index]) // 左下边
                index++;
                if (index >= n) break;
                screen_pos_to_world_pos(x_0 + height - 1 + i,y_0 + size - 1 - i,form_arr[index]) // 右下边
                index++;
            }
            //console.log("neibu",index,n)
            // 填充内部   实际走不到填充的部分
            for ( i = 1; i < size - 1; ++i) {
                for (var j = 1; j < size - 1; ++j) {
                    if (index >= n) {
                        break;
                    }
                   // if(i < height){
                    //    if(j > height - 1 - i && j < height - 1 + i) {
                    //        console.log(index,i,j)
                     //       screen_pos_to_world_pos(x_0 + j,y_0 + i,form_arr[index])
                     //       index++;
                    //    }
                   // }
                    screen_pos_to_world_pos(x_0 + j,y_0 + i,form_arr[index])
                    index++;
                }
            }



     /*   var index = 0;

        var a, b, c;//行数，输出次数，空格数
        var size = 1;
        while(size* size / 2 <= n) {
                        size++
                    }
            if (n % 2 == 1)//n为奇数才能输出完整的菱形，
            {
                for (a = 1; a <= size; a++)//上半部分输出行数
                {
                    c = 1;//空格数
                    // for (b = 0; b < a + (n / 2); b++)//前半部分每行输出次数n/2+1~n次
                    // {
                    //     if (c <= (n / 2 + 1) - a)//判断是否输出空格，否则输出*
                    //     {
                    //      //   printf(" ");
                    //         c++;
                    //     }
                    //     else
                    //     {
                    //         if (index >= n) break;
                    //         screen_pos_to_world_pos(x_0 + b,y_0 + a,form_arr[index++])
                    //       //  printf("*");
                    //     }
                    // }
                    for (b = 0; b < a * 2 - 1; b++)//前半部分每行输出次数n/2+1~n次
                    {
                        {
                            if (index >= n) break;
                            screen_pos_to_world_pos(x_0 + b,y_0 + a,form_arr[index++])
                        }
                    }
                }
                for (a = 1; a <= (n / 2); a++)//后半部分输出行数
                {
                    c = 1;
                    for (b = 1; b <= n - a; b++)//后半部分每行输出次数
                    {
                        {
                            if (index >= n) break;
                            screen_pos_to_world_pos(x_0 + b,y_0 + a + Math.floor(n / 2 + 1),form_arr[index++])
                        }
                    }
                }
            }
            else//输入的n为偶数，菱形不是完整的，按照n-1行输出
            {
                for (a = 1; a <= (n / 2); a++)//前半部分输出行数的循环
                {
                    c = 1;
                    for (b = 1; b <= a + (n / 2 - 1); b++)//每行输出次数循环
                    {
                        if (c <= (n / 2) - a)
                        {
                            c++;
                        }
                        else
                        {
                            if (index >= n) break;
                            screen_pos_to_world_pos(x_0 + b,y_0 + a,form_arr[index++])
                        }
                    }
                }
                for (a = 1; a <= (n / 2 - 1); a++)//后半部分输出行数循环
                {
                    c = 1;
                    for (b = 1; b <= (n - 1) - a; b++)//没行输出次数的循环
                    {
                        if (c <= a)
                        {
                            c++;
                        }
                        else
                        {
                            if (index >= n) break;
                            screen_pos_to_world_pos(x_0 + b,y_0 + a + Math.floor(n / 2),form_arr[index++])

                        }
                    }
                }


            }
            */
/*
        screen_pos_to_world_pos(x_0,y_0,form_arr[0])
        screen_pos_to_world_pos(x_0 - 1,y_0 + 2,form_arr[1])
        screen_pos_to_world_pos(x_0 + 1,y_0 + 2,form_arr[2])
        screen_pos_to_world_pos(x_0 ,y_0 + 4,form_arr[3])*/

        send_all_airplane_pos(input4.text,0)
    }
    // 圆形
    function circle_swarm() {
        form_arr.length = 0
        for(var i = 0; i < plan_arr.length; i++) {
            if (plan_arr[i].group_id === Number(input4.text)) {
                form_arr.push(plan_arr[i])
            }
        }
/*        if (form_arr.length === 3) {
            console.log("本组飞机数量 airplane count :",form_arr.length)
            triangle_swarm()
            return
        }

        if (form_arr.length === 4) {
            console.log("本组飞机数量 airplane count :",form_arr.length)
            rectangle_swarm()
            return
        }

        if (form_arr.length != 8) {
            console.log("airplane count :",form_arr.length)
            return
        }
*/
        var x_1 = 2
        var y_1 = 1
        if ( Number(input4.text) === 1) {
            x_1 = 2
            y_1 = 1
        }
        if ( Number(input4.text) === 2) {
            x_1 = 19
        }
        if ( Number(input4.text) === 3) {
            x_1 = 2
            y_1 = 10
        }
        if ( Number(input4.text) === 4) {
            x_1 = 19
            y_1 = 10
        }


            var n = form_arr.length;
          /*  if (n < 4) {
                return;
            }*/

        /*    // 计算圆的半径
            var radius = n / (2 * Math.PI); // 假设每个占据一个单位弧长
            var diameter = Math.floor(2 * radius) + 1;
          //  var diameter = 2 * radius;

            // 中心点坐标
            var centerX = diameter / 2;
            var centerY = diameter / 2;

            // 均匀分布在圆的周长上
            var index = 0;
            for (var angle = 0; angle < 2 * Math.PI; angle += 2 * Math.PI / n) {
                var x = Math.ceil(centerX + radius * Math.cos(angle));
                var y = Math.ceil(centerY + radius * Math.sin(angle))
               // y = Math.floor(centerY + radius * Math.sin(angle)) < centerY + radius * Math.sin(angle) ?
                  //          Math.ceil(centerY + radius * Math.sin(angle)) - 1 : Math.floor(centerY + radius * Math.sin(angle));

                if (x <= diameter && y <= diameter) {
                    if(index >= n) break
                    console.log("rx ry:",x_1,y_1," r ",centerX)
                    console.log(form_arr[index].objectName,x,y,angle,radius * Math.cos(angle),radius * Math.sin(angle))
                    screen_pos_to_world_pos(x_1 + x,y_1 + y,form_arr[index++])
                }
            }*/


            var radius = n / 2; // 半径不能太小
            if(radius < 2) radius = 2
            if(radius >= 8) radius = 8
            var angleStep = 2 * Math.PI / n; // 每个点之间的角度差
            var index = 0

            for (i = 0; i < n; ++i) {
                var angle = i * angleStep;
                var x = radius + radius * Math.cos(angle);
                var y = radius + radius * Math.sin(angle);
                if(index >= n) break
              /*  if(x - Math.floor(x) > 0.5)
                    x = Math.ceil(x)
                else
                    x = Math.floor(x)
                if(y - Math.floor(y) > 0.5)
                    y = Math.ceil(y)
                else
                    y = Math.floor(y)*/
                screen_pos_to_world_pos(x_1 + x,y_1 + y,form_arr[index++])
            }


/*
        screen_pos_to_world_pos(x_1,y_1,        form_arr[0])
        screen_pos_to_world_pos(x_1 + 2,y_1,    form_arr[1])
        screen_pos_to_world_pos(x_1 - 1,y_1 + 1,form_arr[2])
        screen_pos_to_world_pos(x_1 + 3,y_1 + 1,form_arr[3])
        screen_pos_to_world_pos(x_1,    y_1 + 4,form_arr[4])
        screen_pos_to_world_pos(x_1 + 2,y_1 + 4,form_arr[5])
        screen_pos_to_world_pos(x_1 - 1,y_1 + 3,form_arr[6])
        screen_pos_to_world_pos(x_1 + 3,y_1 + 3,form_arr[7])
*/
        send_all_airplane_pos(input4.text,0)
    }

    // 直线型（内置队形）  支持所有机体数量，有几个排几个
    function stright_line_swarm() {

        form_arr.length = 0
        for(var i = 0; i < plan_arr.length; i++) {
            if (plan_arr[i].group_id === Number(input4.text)) {
                form_arr.push(plan_arr[i])
            }
        }
       /* if (form_arr.length !== 4) {
            console.log("本组飞机数量 airplane count :",form_arr.length)
            return
        }*/
        var x_0 = 0
        var y_0 = 1
        if (Number(input4.text) === 1) { // 横坐标起始点（基准线）
            x_0 = 1
        } else if(Number(input4.text) === 2) {
            x_0 = 18
            y_0 = 1
        } else if(Number(input4.text) === 3) {
            x_0 = 1
            y_0 = 10
        } else if(Number(input4.text) === 4) {
            x_0 = 18
            y_0 = 10
        }


        var dis = 0
        for(var i1 = 0; i1 < form_arr.length; i1++) {
            while (transform_crush(x_0 + dis,y_0,form_arr[i1].group_id)) {
                x_0 += 2
            }
            screen_pos_to_world_pos(x_0 + dis, y_0, form_arr[i1])
            dis += 1
        }
// 口述：飞行中点东西一字型   有一个没在队形里面，且主机的颜色显示变化了，且飞机的序号2号把4号覆盖了，两个2号了
        send_all_airplane_pos(input4.text,0)
    }

    function stright_NS_line_swarm() {
        form_arr.length = 0
        for(var i = 0; i < plan_arr.length; i++) {
            if (plan_arr[i].group_id === Number(input4.text)) {
                form_arr.push(plan_arr[i])
            }
        }
       /* if (form_arr.length !== 4) {
            console.log("本组飞机数量 airplane count :",form_arr.length)
            return
        }*/
        var x_0 = 0
        var y_0 = 2
        if (Number(input4.text) === 1) { // 横坐标起始点（基准线）
            x_0 = 1
        } else if(Number(input4.text) === 2) {
            x_0 = 18
        } else if(Number(input4.text) === 3) {
            x_0 = 1
            y_0 = 10
        } else if(Number(input4.text) === 4) {
            x_0 = 18
            y_0 = 10
        }


        var dis = 0
        for(var i2 = 0; i2 < form_arr.length; i2++) {
            while (transform_crush(x_0,y_0 + dis,form_arr[i2].group_id)) {
                y_0 += 1
            }
            screen_pos_to_world_pos(x_0, y_0 + dis, form_arr[i2])
                dis += 2
        }

        send_all_airplane_pos(input4.text,0)
    }
}
