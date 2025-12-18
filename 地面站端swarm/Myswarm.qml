import QtQuick

import QtQuick.Window
import QtQuick.Controls
import QtPositioning
import QtQuick3D
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
Window {
    id:root
    title: "我的编队"
    height:810
    width:1360
    x:300
    y:300

    signal message()
    visible:false
    signal update_other_airplane(int param,int isset,int grp)

    Mavlinktest2 {
       id: test_mavlink
    }
    Swarmsend {
        id: swarm_send
    }

    color: "darkred"

    property var    _activeVehicle:     QGroundControl.multiVehicleManager // 好像没用了  cpp函数可以删了
    property var    _sysid_list: []
    property var    idpos_map: {0:0}
    property var    hasset_map: {0:0}
    onUpdate_other_airplane: function(par,set,gr){ // 只在主机更新时,同组的有些模型还没加载出来,没加载出来的模型的位置会遗漏

                if (par === 2 && set === 0) { //只更新位置,所有
                    update_all_pos()
                } else if (par === 2 && set === 1) {// 是最后一个且是主机
                    for (var j = 0; j < plan_arr.length; j++) {
                        if (plan_arr[j].is_connected && plan_arr[j].set_main) {
                                set_main_behavior(plan_arr[j],par)
                        }
                    }
                }
                if (par === 1) {// 是主机
                    for (var h = 0; h < plan_arr.length; h++) {
                        if(plan_arr[h].is_connected && plan_arr[h].set_main) {
                            set_main_behavior(plan_arr[h],0) // 不发位置
                        }
                    }
                }

    }
    function my_delay(){
        for(var i = 0;i<2000;i++){
            for(var j = 0; j <2000;j++){

            }
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
                        swarm_send.caculate_pos(_sysid_list[n], 0, 0, 0)
                        continue;
                    }
                    if(send_f)for(var k  = 0; k < 10000000;k++){}// 避免数据拥堵
                    swarm_send.caculate_pos(_sysid_list[n],
                                              -(idpos_map[_sysid_list[n]][1] - idpos_map[main_node_name[grp_n - 1]][1]) * input1.text,
                                              (idpos_map[_sysid_list[n]][0] - idpos_map[main_node_name[grp_n - 1]][0]) * input1.text,
                                              -(idpos_map[_sysid_list[n]][2] - idpos_map[main_node_name[grp_n - 1]][2]))
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
    function set_main_color(node) {
        for (var n = 0; n < plan_arr.length; n++) {
            if (plan_arr[n].group_id === node.group_id && plan_arr[n].objectName !== node.objectName) { // 只对这一组的颜色进行排他
                if (if_main_node(plan_arr[n].objectName) ) {
                    plan_arr[n].is_main = true
                    continue;
                }
                plan_arr[n].is_main = false
         //       console.log("set main_color ",plan_arr[n].id,plan_arr[n].objectName)
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
    function move_model(n,sumn) {// n 指屏幕位置
        var xx = 0
        var yy = 0
        var mstart = 0
        var lim = 0

        if (sumn === 2 && n === 1) lim = (root.width -20) / 80 // 第一组的界限
        if ((sumn === 2 && n === 2) || (sumn === 4 && n === 2)) {
            mstart = (root.width) / 80;
            lim = (root.width -20) / 40;
            xx = mstart
        }
        if (sumn === 4 && n === 3) {lim = (root.width -20) / 80; yy = (root.height - 20) / 80}
        if (sumn === 4 && n === 4) {mstart = (root.width) / 80; lim = (root.width -20) / 40; xx = mstart;yy = (root.height - 20) / 80}
     //   console.log("move ",group_num,n,xx,yy,lim,trans_pos_to_grp(n))
        for (var i = 0; i < plan_arr.length; i++) {
            if (plan_arr[i].group_id === Number(trans_pos_to_grp(n))) { //这个位置 转换成组别
     //           console.log(xx,yy,lim)
                if(xx < lim) {
                    screen_pos_to_world_pos(xx,yy,plan_arr[i])
                    xx++
                } else {
                    yy++
                    xx = mstart
                    screen_pos_to_world_pos(xx,yy,plan_arr[i])
                    xx++
                }
            }
        }
    }
    // onMyIntxChanged:checkvalues()
    // onMyIntyChanged:checkvalues()
    // function checkvalues() {
    //     console.log(myIntx,myInty)
    // }
    property bool ifpick: false
    property int myIntx:0
    property int myInty:0
    property int lastIntx: 0
    property int lastInty: 0
    property bool if_release: false
  //  property real num1:_activeVehicle.Vehicle_count // 好像没用了
    // property string main_node_name: ""
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

    Button{
        id:bt1
        height:30
        width:70
        x:0
        y:10
        text: "自动"
        visible: true
        onClicked: {
   //         num1 = -num1
   //         mygrid.itemAt(530,30).setText(num1)
        //    myqmlsig(520,"")
            test_mavlink._sendcom(1,0,0,0,0)

           // console.log(idpos_map[20],idpos_map[1],"----")
        }
    }
//
    Button{
        id:bt2
        x:80
        y:10
        height:30
        width:70
        text: "开始"
        visible: true
        onClicked: {
            test_mavlink._sendcom(0,1,0,0,0)
        }
    }
    Button{
        id:bt3
        x:240
        y:10
        height:30
        width:70
        text: "选定主机"
        visible: true
        onClicked: {
            if (mouse_area.pickNode === null) {
                return
            }
            mouse_area.pickNode.set_main = 1   // 需对同组其他主机进行排他
          //  plan_to_out_main(mouse_area.pickNode)

            // 如果选中了  或者重复选中   分组后， 选主机  移动4 相对位置没有变， 选主机1 移动1相对位置变换正常
           // if (mouse_area.pickNode || (mouse_area.lastpickNode === mouse_area.pickNode && mouse_area.lastpickNode != null)) {
                    main_node_name[mouse_area.pickNode.group_id - 1] = mouse_area.pickNode.objectName
            set_main_name(mouse_area.pickNode)
             //   console.log(mouse_area.pickNode.group_id - 1,mouse_area.pickNode.objectName, main_node_name[mouse_area.pickNode.group_id - 1])
                //    mouse_area.lastpickNode = mouse_area.pickNode
                if (!mouse_area.pickNode.is_connected) return
                mouse_area.pickNode.is_main = true
              //  swarm_send.set_main_airplane(main_node_name[node.group_id - 1], node.group_id, 0, 0, 0)
                set_main_behavior(mouse_area.pickNode,1)  //要发位置
                /*
                    swarm_send.set_main_airplane(main_node_name[mouse_area.pickNode.group_id - 1], mouse_area.pickNode.group_id,
                                                 idpos_map[main_node_name[mouse_area.pickNode.group_id - 1]][0],
                                                 idpos_map[main_node_name[mouse_area.pickNode.group_id - 1]][1],
                                                 idpos_map[main_node_name[mouse_area.pickNode.group_id - 1]][2])
                    swarm_send.caculate_pos(mouse_area.pickNode.objectName, 0, 0, 0)
                    send_all_airplane_pos()
                    set_main_color(mouse_area.pickNode.objectName)*/
          //  }

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
    Button{
        id:bt4
        x:160
        y:10
        height:30
        width:70
        text: "停止"
        visible: true
        onClicked: {
            test_mavlink._sendcom(0,0,1,0,0)
        /*    for(var i =0;i< main_node_name.length;i++) {
                console.log("all main",main_node_name[i])
            }*/
        }
    }
    Button{
        id:bt5
        x:540
        y:10
        height:30
        width:50
        text: "分组"
        visible: true
        Popup {
            id: popup
            x: 100
            y: 50
            width: 200
            height: 100
            modal: true // 使弹出窗口为模态
            focus: true // 设置弹出窗口获得焦点

            // 弹出窗口内容
            contentItem: Text {
                text: qsTr("超出机架数量范围，请输入合理值!")
                wrapMode: TextEdit.Wrap
                color: "black"
                font.pixelSize: 14
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }

            // 连接按钮点击信号，显示弹出窗口
            Connections {
                target: bt5
                function onPressed() {
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

                            move_model(1,2)
                            move_model(2,2)
                        } else if (input3.text === "3") {
                            canv.visible = true
                            canv4.visible = true
                            canv2.visible = true
                            canv3.visible = true
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
                            move_model(1,2)
                            move_model(2,2)
                            move_model(3,4)
                        } else if (input3.text === "4") {
                            canv.visible = true
                            canv4.visible = true
                            canv2.visible = true
                            canv3.visible = true
                            for (var l = 0; l < plan_arr.length; l++) {
                                if (l < plan_arr.length / 4) {
                                    plan_arr[l].group_id = 1
                                    i = l
                                } else if (l >= plan_arr.length / 4 && l < plan_arr.length / 2) {
                                    plan_arr[l].group_id = 2
                                    j = l
                                } else if (l >= plan_arr.length / 2 && l < plan_arr.length * 3 / 4) {
                                    plan_arr[l].group_id = 3
                                    k = l
                                } else {
                                    plan_arr[l].group_id = 4
                                }
                                if(plan_arr[l].is_connected)swarm_send.store_airplane_group(plan_arr[l], plan_arr[l].group_id, true)
                            }
                            main_node_name.length = 4

                            for (var mn4_1 = 0; mn4_1 < plan_arr.length  / 4; mn4_1++) {
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

                            for (var mn4_2 = i + 1; mn4_2 >= plan_arr.length / 4 && mn4_2 < plan_arr.length / 2; mn4_2++) {
                                if (if_main_node(plan_arr[mn4_2].objectName)) {
                                    main_node_name[1] = plan_arr[mn4_2].objectName
                                    set_main_name(plan_arr[mn4_2])
                                    if(plan_arr[mn4_2].is_connected === true)set_main_color([plan_arr[mn4_2]].objectName)
                                    break;
                                }
                                main_node_name[1] = plan_arr[mn4_2].objectName
                                set_main_name(plan_arr[mn4_2])
                                if(plan_arr[mn4_2].is_connected === true)set_main_color(plan_arr[mn4_2].objectName)
                            }

                            for (var mn4_3 = j + 1; mn4_3 >= plan_arr.length / 2 && mn4_3 < plan_arr.length *3 / 4; mn4_3++) {
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
                            for (var mn4_4 = k + 1; mn4_4 >= plan_arr.length *3 / 4 && mn4_4 < plan_arr.length; mn4_4++) {
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

                          /*  grp_pos_mp[1] = 1 // 如果是2345 组呢
                            grp_pos_mp[2] = 2
                            grp_pos_mp[3] = 3
                            grp_pos_mp[4] = 4*/
                            for(i = 0; i < main_node_name.length;i++) {
                                for(j = 0;j < plan_arr.length;j++){
                                    if(plan_arr[j].objectName === main_node_name[i]) {

                                        grp_pos_mp[plan_arr[j].group_id] = i + 1 // 要的是grp  不是name
                                    }
                                }
                            }
                            move_model(1,2)
                            move_model(2,2)
                            move_model(3,4)
                            move_model(4,4)
                        }
                    }
                }
            }
        }
    }
    Rectangle{
        x:590
        y:10
        height:30
        width:20
        color: "linen"
        border.color: "lightgray"
        TextEdit{
            y:7
            id:input3
            width: 20
            text: "1"
          //  wrapMode: TextEdit.Wrap
            clip: true // 防止超出范围

            onTextChanged: {
                // 如果输入的文本宽度超过了TextEdit的内容区域宽度，则截断文本
                if (contentWidth > width) {
                    text = text.slice(0, -1) // 移除最后一个字符
                }
            }
        }
    }

    Label{
        x:320
        y:15
        width: 50
        text: "设置间距"
        style: Text.Outline; // 可选的样式有Text.Normal, Text.Outline, Text.Raised等
        color: "lightgreen";       // 正常颜色
     //   outlineColor: "red";  // 边缘或描边的颜色
        font.bold: true       // 可选的文本样式
    }
    Rectangle{
        x:370
        y:10
        height:30
        width:20
        color: "linen"
        border.color: "lightgray"
        TextEdit{
            y:7
            id:input1
            width: 20
            text: "1"
          //  wrapMode: TextEdit.Wrap
            clip: true // 防止超出范围

            onTextChanged: {
                // 如果输入的文本宽度超过了TextEdit的内容区域宽度，则截断文本
                if (contentWidth > width) {
                    text = text.slice(0, -1) // 移除最后一个字符
                }
            }
        }
    }
    Label{
        x:390
        y:15
        width: 40
        text: "米"
        style: Text.Outline; // 可选的样式有Text.Normal, Text.Outline, Text.Raised等
        color: "lightgreen";       // 正常颜色
        font.bold: true       // 可选的文本样式
    }

    Button{
        x:410
        y:10
        width: 70
        height: 30
        text: "设置高度"
        onClicked: {
            if (!mouse_area.pickNode) {
                return
            }
            mouse_area.pickNode.model_z = Number(input2.text)
            if (mouse_area.pickNode.is_connected) {
                idpos_map[mouse_area.pickNode.objectName][2] = Number(input2.text)
            }

            show_position(mouse_area.pickNode) // 界面显示数据
            send_all_airplane_pos(mouse_area.pickNode.group_id,0)
        }
    }
    Rectangle{
        x:480
        y:10
        height:30
        width:20
        color: "linen"
        border.color: "lightgray"
        TextEdit{
            y:7
            id:input2
            width: 20
            text: "0"
          //  wrapMode: TextEdit.Wrap
            clip: true // 防止超出范围

            onTextChanged: {
                // 如果输入的文本宽度超过了TextEdit的内容区域宽度，则截断文本
                if (contentWidth > width) {
                    text = text.slice(0, -1) // 移除最后一个字符
                }
            }
        }
    }
    Label{
        x:500
        y:15
        width: 30
        text: "米"
        style: Text.Outline; // 可选的样式有Text.Normal, Text.Outline, Text.Raised等
        color: "lightgreen";       // 正常颜色
        font.bold: true       // 可选的文本样式
    }

    Label{
        x:625
        y:15
        width: 30
        text: "第"
        style: Text.Outline; // 可选的样式有Text.Normal, Text.Outline, Text.Raised等
        color: "lightgreen";       // 正常颜色
        font.bold: true       // 可选的文本样式
    }
    Rectangle{
        x:640
        y:10
        height:30
        width:20
        color: "linen"
        border.color: "lightgray"
        TextEdit{
            y:7
            id:input4  //第几组   队形变换
            width: 20
            text: "1"
          //  wrapMode: TextEdit.Wrap
            clip: true // 防止超出范围

            onTextChanged: {
                // 如果输入的文本宽度超过了TextEdit的内容区域宽度，则截断文本
                if (contentWidth > width) {
                    text = text.slice(0, -1) // 移除最后一个字符
                }
            }
        }
    }
    Label{
        x:662
        y:15
        width: 30
        text: "组"
        style: Text.Outline; // 可选的样式有Text.Normal, Text.Outline, Text.Raised等
        color: "lightgreen";       // 正常颜色
        font.bold: true       // 可选的文本样式
    }
    ComboBox {
            id: comboBox
            x: 680
            y: 10
            width: 95
            height: 30

            model: ["队形变换","东西一字形","南北一字形", "三角队形", "正方队形","菱形队形","圆形队形"] // 下拉框的选项
            onActivated:  {
                // 当选项变更时的处理
                console.log("选中的选项:", currentText)
                if (currentText === "东西一字形") {
                    stright_line_swarm()
                } else if(currentText === "南北一字形") {
                    stright_NS_line_swarm()
                } else if(currentText === "三角队形") {
                    triangle_swarm()
                } else if (currentText === "正方队形") {
                    rectangle_swarm()
                } else if (currentText === "菱形队形") {
                    diamond_swarm()
                } else if (currentText === "圆形队形") {
                    circle_swarm()
                }
            }
        }
    Button{
        x:780
        y:10
        width: 70
        height: 30
        text: "更换分组"

        Popup {
            id: group_popup
            x: 0
            y: 50
            width: 200
            height: 100
            modal: true // 使弹出窗口为模态
            focus: true // 设置弹出窗口获得焦点

            // 弹出窗口内容
            contentItem: Text {
                text: qsTr("此分组数值目前状态不可用!")
                wrapMode: TextEdit.Wrap
                color: "black"
                font.pixelSize: 14
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }
        Popup {
            id: group_popup2
            x: 0
            y: 50
            width: 200
            height: 100
            modal: true // 使弹出窗口为模态
            focus: true // 设置弹出窗口获得焦点

            // 弹出窗口内容
            contentItem: Text {
                text: qsTr("请先选中待更改分组的模型!")
                wrapMode: TextEdit.Wrap
                color: "black"
                font.pixelSize: 14
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }
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

            } else {
                group_popup.open()
            }
        }
    }
    Rectangle{
        x:850
        y:10
        height:30
        width:20
        color: "linen"
        border.color: "lightgray"
        TextEdit{
            y:7
            id:input_change
            width: 20
            text: "1"
          //  wrapMode: TextEdit.Wrap
            clip: true // 防止超出范围

            onTextChanged: {
                // 如果输入的文本宽度超过了TextEdit的内容区域宽度，则截断文本
                if (contentWidth > width) {
                    text = text.slice(0, -1) // 移除最后一个字符
                }
            }
        }
    }


    Button{
        x:880
        y:10
        width: 70
        height: 30
        text: "暂停"
        onClicked: {
            test_mavlink._sendcom2(0,0,0,separate_main,0)
        }
    }
    Button{
        x:960
        y:10
        width: 70
        height: 30
        text: "继续"
        onClicked: {
            test_mavlink._sendcom(0,0,0,0,separate_main)
        }
    }
    Button{
        x:1040
        y:10
        width: 70
        height: 30
        text: "筹划"
        onClicked: {
            if (Number(input_plan.text) > 50) {
                group_popu4.open()
            } else
            plan_to_visible()
        }

        Popup {
            id: group_popu4
            x: 0
            y: 50
            width: 200
            height: 100
            modal: true // 使弹出窗口为模态
            focus: true // 设置弹出窗口获得焦点

            // 弹出窗口内容
            contentItem: Text {
                text: qsTr("筹划上限为50架!")
                wrapMode: TextEdit.Wrap
                color: "black"
                font.pixelSize: 14
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }
    }
    Rectangle{
        x:1100
        y:10
        height:30
        width:20
        color: "linen"
        border.color: "lightgray"
        TextEdit{
            y:7
            id:input_plan
            width: 20
            text: "0"
          //  wrapMode: TextEdit.Wrap
            clip: true // 防止超出范围

            onTextChanged: {
                // 如果输入的文本宽度超过了TextEdit的内容区域宽度，则截断文本
                if (contentWidth > width) {
                    text = text.slice(0, -1) // 移除最后一个字符
                }
            }
        }
    }
    Label{
        x:1120
        y:15
        width: 30
        text: "架"
        style: Text.Outline; // 可选的样式有Text.Normal, Text.Outline, Text.Raised等
        color: "lightgreen";       // 正常颜色
        font.bold: true       // 可选的文本样式
    }

    Button{
        x:1150
        y:10
        width: 70
        height: 30
        text: "独立分组"
        onClicked: {  // 合并再分组时,数字变了,但没有写入,没有发送?更新时的主机数量不对
            if(select_merge.length === 0)return
            if(group_num >= 4) {
                select_merge.length = 0;
                for(var i1 = 0; i1 < select_merge.length; i1++) {
                    select_merge[i1].select_color = 0.6 // 颜色咋没恢复 ?
                }
                devide_grp_pop.open();
                return
            }
            group_num += 1

            for(var i = 0; i < select_merge.length; i++) {
                select_merge[i].group_id = main_node_name.length + 1//默认main_node[]是连续的,如果不连续  会出问题   能不能用main_node[]length?
              //  console.log("grp",select_merge[i].group_id)
                select_merge[i].select_color = 0.6

                if(select_merge[i].is_connected === true)swarm_send.store_airplane_group(select_merge[i].objectName, select_merge[i].group_id,true)
              /*  else
                    swarm_send.store_airplane_group(select_merge[i].objectName, select_merge[i].group_id,false) // 应该不影响
                */
            }

          //  main_node_name.length = 1
            main_node_name[main_node_name.length]=select_merge[0].objectName
         //   for(var u = 0; u < main_node_name.length;u++)console.log("after merge main",u,main_node_name[u])
         //   console.log("se  len",select_merge.length,group_num,main_node_name[group_num-1],select_merge[0].objectName)
            select_merge[0].set_main = 1
         //   set_main_name(select_merge[0])
            if(select_merge[0].is_connected === true){
                swarm_send.set_main_airplane(main_node_name[main_node_name.length - 1], select_merge[0].group_id,
                                  0,
                                  0,
                                  0)


                select_merge[0].is_main = true
                set_main_color(select_merge[0])
                send_all_airplane_pos(main_node_name.length,0)
            }
            devide_screen(select_merge[0].group_id)
            select_merge.length = 0

            all_move_by_line()
        }

        Popup {
            id: devide_grp_pop
            x: 0
            y: 50
            width: 200
            height: 100
            modal: true // 使弹出窗口为模态
            focus: true // 设置弹出窗口获得焦点

            // 弹出窗口内容
            contentItem: Text {
                text: qsTr("分组上限为4组，请合理设置分组!")
                wrapMode: TextEdit.Wrap
                color: "black"
                font.pixelSize: 14
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }
    }
    Button{
        x:1240
        y:10
        width: 70
        height: 30
        text: "合并分组"  // 需解决 位置改变  计算原组别的最右或最下角的位置  (如果是1 2组找最下边,如果是 ),找带和入的最左边和最上面,移动模型,如果有与其他组的模型重合则移动失败,一般判断1 4,2 3
        onClicked: {
            if(select_merge.length < 1)return
            for(var c = 0; c < select_merge.length;c++){if(select_merge[c].set_main !== true)return}
            var temp = 0
            for(var h = 1; h < select_merge.length; h++) {
                temp = select_merge[h].group_id
                main_node_name[temp - 1] = 0

                find_max_pos(select_merge[0].group_id, select_merge[h].group_id)  // 如果是剩余两组呢？
                grp_pos_mp[select_merge[h].group_id] = 0
                delete grp_pos_mp[select_merge[h].group_id]
            }
            merge_grp()
            for(var i = 1; i < select_merge.length; i++) {


              //  console.log("grp",i,select_merge[i].objectName,select_merge[i].group_id,main_node_name[temp - 1])

                select_merge[i].group_id = select_merge[0].group_id // 全设为第一个选中的
                select_merge[i].set_main = 0
                if (select_merge[i].is_connected === false)select_merge[i].is_main = 0

                select_merge[i].select_color = 0.6

            }
            select_merge[0].select_color = 0.6
          //  main_node_name.length = 1

            group_num = group_num - select_merge.length + 1

          //  set_main_name(select_merge[0])
            if(select_merge[0].is_connected === true){
      /*          swarm_send.set_main_airplane(main_node_name[group_num - 1], select_merge[0].group_id,// 还需要吗
                                  0,
                                  0,
                                  0)
*/
              //  select_merge[0].is_main = true
              //  set_main_color(select_merge[0])
                send_all_airplane_pos(select_merge[0].group_id,0)
            }
          //  devide_screen(select_merge[0].group_id)
            select_merge.length = 0
         //   for(var u = 0; u < main_node_name.length;u++)console.log("after merge main",u,main_node_name[u])
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

    Rectangle {
        height:root.height - 50
        width:root.width
        y:50
        id:rc
        focus: true

        View3D {
            id: control
            anchors.fill:parent
            //背景
            environment: SceneEnvironment {
             //   clearColor: "darkGreen"
                clearColor: "skyblue"
                backgroundMode: SceneEnvironment.Color
             //   backgroundMode: SceneEnvironment.Transparent
            }

            // Coordinate3D {
            //        grid: true // 显示网格线
            //        ticks: 11 // 刻度线数量
            //        size: 200.0 // 坐标系大小
            //    }

                GridView{
                    id:mygrid
                    anchors.fill: parent
                    anchors.margins: 0

                    clip: true　　// 设置clip属性为true，来激活裁剪功能

                    model:1500
                    delegate: numberDelegate
                    cellHeight: 40
                    cellWidth: 40
                //   color: SceneEnvironment.Transparent
                }
                Component{
                    id:numberDelegate
                    Rectangle{
                        id:rct
                        width: 40;
                        height: 40;
                      //  color: "lightGreen"
                        color: "Transparent"
                        Text {
                            id:txt
                            anchors.centerIn: parent
                            font.pixelSize: 15
                            text: "+"

                          //  text: (index < 12) || (index % 12 == 0) ? index: "+"

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
                z:   control.height * 3.5   //1800左右
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
                    var startX = root.width / 2
                    var startY = 0

                    var endX = root.width / 2
                    var endY =(root.height - 50) / 2

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
                    var startX = root.width / 2
                    var startY = (root.height - 50) / 2

                    var endX = root.width / 2
                    var endY = root.height

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
                    var startY = (root.height - 50) / 2

                    var endX = root.width / 2
                    var endY = (root.height - 50) / 2

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
                    var startX = root.width / 2
                    var startY = (root.height - 50) / 2

                    var endX = root.width
                    var endY = (root.height - 50) / 2

                    vtx.beginPath()
                    vtx.moveTo(startX,startY)
                    vtx.lineTo(endX,endY)
                    vtx.stroke()
                }
            }
            /*立方体
            Model {
                }
                //立方体转动
               /* SequentialAnimation on eulerRotation {
                    running: true
                    loops: Animation.Infinite
                    PropertyAnimation {
                        duration: 10000
                        from: Qt.vector3d(0, 0, 0)
                        to: Qt.vector3d(360, 360, 360)
                    }
                }*/

            //球体
            Loader {
                        id: modelLoader

                        active: false // 初始不加载模型
                        x: 180// % 40 > 20 ?
                        y: 0
                        z: 50
                        // 在Loader被激活时加载以下资源
                        source: "qrc:/qml/MyFlyEnvironment.qml" // 假设这是一个3D模型的QML资源路径

                        // 如果需要在加载时执行额外操作，可以在onLoaded和onLoadingChanged处理
                        onLoaded: {
                            console.log("loaded.............")
                            // 模型加载完成后执行的代码
                        }
                        function loadModel() {
                            active = true;
                         //   item.entity.visible = true
                        }
            }

            //children:[]
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node.is_connected ? (sphere_node.is_main ? "red" : "cyan") : "#646566"

                }
               // materials: edgeColor:Qt.rgba(1.0,0.0,0.0,1.0)
              //  materials:
                Component.onCompleted: {
                    mymove(1,1,sphere_node) // 在这去更新 x y z
                    get_pos(sphere_node);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node2.is_connected ? (sphere_node2.is_main ? "red" : "cyan") : "#646566"
                }
                Component.onCompleted: {
                    mymove(2,1,sphere_node2)
                    get_pos(sphere_node2);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node3.is_connected ? (sphere_node3.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(3,1,sphere_node3)
                    get_pos(sphere_node3);

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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node4.is_connected ? (sphere_node4.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(4,1,sphere_node4)
                    get_pos(sphere_node4);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node5.is_connected ? (sphere_node5.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(5,1,sphere_node5)
                    get_pos(sphere_node5);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node6.is_connected ? (sphere_node6.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(6,1,sphere_node6)
                    get_pos(sphere_node6);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node7.is_connected ? (sphere_node7.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(7,1,sphere_node7)
                    get_pos(sphere_node7);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node8.is_connected ? (sphere_node8.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(8,1,sphere_node8)
                    get_pos(sphere_node8);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node9.is_connected ? (sphere_node9.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(9,1,sphere_node9)
                    get_pos(sphere_node9);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node10.is_connected ? (sphere_node10.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(10,1,sphere_node10)
                    get_pos(sphere_node10);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node11.is_connected ? (sphere_node11.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(11,1,sphere_node11)
                    get_pos(sphere_node11);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node12.is_connected ? (sphere_node12.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(12,1,sphere_node12)
                    get_pos(sphere_node12);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node13.is_connected ? (sphere_node13.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(13,1,sphere_node13)
                    get_pos(sphere_node13);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node14.is_connected ? (sphere_node14.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(14,1,sphere_node14)
                    get_pos(sphere_node14);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node15.is_connected ? (sphere_node15.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(15,1,sphere_node15)
                    get_pos(sphere_node15);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node16.is_connected ? (sphere_node16.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(16,1,sphere_node16)
                    get_pos(sphere_node16);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node17.is_connected ? (sphere_node17.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(17,1,sphere_node17)
                    get_pos(sphere_node17);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node18.is_connected ? (sphere_node18.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(18,1,sphere_node18)
                    get_pos(sphere_node18);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node19.is_connected ? (sphere_node19.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(19,1,sphere_node19)
                    get_pos(sphere_node19);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node20.is_connected ? (sphere_node20.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(20,1,sphere_node20)
                    get_pos(sphere_node20);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node21.is_connected ? (sphere_node21.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(21,1,sphere_node21)
                    get_pos(sphere_node21);
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
                scale: Qt.vector3d(1,1,0.1)
                property int model_x : 0
                property int model_y : 0
                property int model_z : 0
                property int group_id: 1
                property bool set_main: false
                property bool is_main: false
                property bool is_connected: false
                property real select_color: 0.6
                materials: DefaultMaterial {
                    diffuseColor: sphere_node22.is_connected ? (sphere_node22.is_main ? "red" : "cyan") : "#646566"
                    opacity: sphere_node22.select_color
                }

                Component.onCompleted: {
                    mymove(22,1,sphere_node22)
                    get_pos(sphere_node22);
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
                scale: Qt.vector3d(1,1,0.1)
                property int model_x : 0
                property int model_y : 0
                property int model_z : 0
                property int group_id: 1
                property bool set_main: false
                property bool is_main: false
                property bool is_connected: false
                property real select_color: 0.6
                materials: DefaultMaterial {
                    diffuseColor: sphere_node23.is_connected ? (sphere_node23.is_main ? "red" : "cyan") : "#646566"
                    opacity: sphere_node23.select_color
                }

                Component.onCompleted: {
                    mymove(23,1,sphere_node23)
                    get_pos(sphere_node23);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node24.is_connected ? (sphere_node24.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(24,1,sphere_node24)
                    get_pos(sphere_node24);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node25.is_connected ? (sphere_node25.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(25,1,sphere_node25)
                    get_pos(sphere_node25)

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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node26.is_connected ? (sphere_node26.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(26,1,sphere_node26)
                    get_pos(sphere_node26);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node27.is_connected ? (sphere_node27.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(27,1,sphere_node27)
                    get_pos(sphere_node27);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node28.is_connected ? (sphere_node28.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(1,2,sphere_node28)
                    get_pos(sphere_node28);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node29.is_connected ? (sphere_node29.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(2,2,sphere_node29)
                    get_pos(sphere_node29);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node30.is_connected ? (sphere_node30.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(3,2,sphere_node30)
                    get_pos(sphere_node30);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node31.is_connected ? (sphere_node31.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(4,2,sphere_node31)
                    get_pos(sphere_node31);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node32.is_connected ? (sphere_node32.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(5,2,sphere_node32)
                    get_pos(sphere_node32);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node33.is_connected ? (sphere_node33.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(6,2,sphere_node33)
                    get_pos(sphere_node33);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node34.is_connected ? (sphere_node34.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(7,2,sphere_node34)
                    get_pos(sphere_node34);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node35.is_connected ? (sphere_node35.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(8,2,sphere_node35)
                    get_pos(sphere_node35);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node36.is_connected ? (sphere_node36.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(9,2,sphere_node36)
                    get_pos(sphere_node36);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node37.is_connected ? (sphere_node37.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(10,2,sphere_node37)
                    get_pos(sphere_node37);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node38.is_connected ? (sphere_node38.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(11,2,sphere_node38)
                    get_pos(sphere_node38);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node39.is_connected ? (sphere_node39.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(12,2,sphere_node39)
                    get_pos(sphere_node39);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node40.is_connected ? (sphere_node40.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(13,2,sphere_node40)
                    get_pos(sphere_node40);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node41.is_connected ? (sphere_node41.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(14,2,sphere_node41)
                    get_pos(sphere_node41);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node42.is_connected ? (sphere_node42.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(15,2,sphere_node42)
                    get_pos(sphere_node42);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node43.is_connected ? (sphere_node43.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(16,2,sphere_node43)
                    get_pos(sphere_node43);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node44.is_connected ? (sphere_node44.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(17,2,sphere_node44)
                    get_pos(sphere_node44);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node45.is_connected ? (sphere_node45.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(18,2,sphere_node45)
                    get_pos(sphere_node45);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node46.is_connected ? (sphere_node46.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(19,2,sphere_node46)
                    get_pos(sphere_node46);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node47.is_connected ? (sphere_node47.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(20,2,sphere_node47)
                    get_pos(sphere_node47);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node48.is_connected ? (sphere_node48.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(21,2,sphere_node48)
                    get_pos(sphere_node48);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node49.is_connected ? (sphere_node49.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(22,2,sphere_node49)
                    get_pos(sphere_node49);
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
                scale: Qt.vector3d(1,1,0.1)
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
                    diffuseColor: sphere_node50.is_connected ? (sphere_node50.is_main ? "red" : "cyan") : "#646566"
                }

                Component.onCompleted: {
                    mymove(23,2,sphere_node50)
                    get_pos(sphere_node50);
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
            //展示拾取对象的信息
         /*   Row {
                x: 20
                y: 20
                spacing: 10
                Column {
                    Label {
                        color: "white"
                        text: "飞机坐标:"
                    }
                    Label {
                        color: "white"
                    //    text: "Screen Position:"
                    }
                }
                Column {
                    Label {
                        id: pick_name
                        color: "white"
                        text: ""
                        property int flyz : 0
                     //   text: mouse_area.pickNode === "一号" ?"":""
                    }
                    Label {
                        id: pick_screen
                        color: "white"
                        text: ""
                    }
                }
                Column {
                    Label {
                        id: pick_name2
                        color: "white"
                        text: ""
                        property int flyz : 0
                    }
                    Label {
                        id: pick_screen2
                        color: "white"
                        text: ""
                    }
                }
                Column {
                    Label {
                        id: pick_name3
                        color: "white"
                        text: ""
                        property int flyz : 0
                    }
                    Label {
                        id: pick_screen3
                        color: "white"
                        text:""
                    }
                }
                Column {
                    Label {
                        id: pick_name4
                        color: "white"
                        text: ""
                        property int flyz : 0
                    }
                    Label {
                        id: pick_screen4
                        color: "white"
                        text:""
                    }
                }
                Column {
                    Label {
                        id: pick_name5
                        color: "white"
                        text: ""
                        property int flyz : 0
                    }
                    Label {
                        id: pick_screen5
                        color: "white"
                        text:""
                    }
                }
            }*/
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
                        if(mouse.x < (root.width ) / 2 && (2 === grp_pos_mp[pickNode.group_id])) {
                            screen_pos_to_world_pos((root.width - 20) / 80,y,pickNode)
                            if_release = false
                        } else if (mouse.x > (root.width ) / 2 && (grp_pos_mp[pickNode.group_id] === 1)){
                            screen_pos_to_world_pos((root.width - 20) / 80 - 1,y,pickNode)
                            if_release = false
                        }
                    }
                    if(canv2.visible === true) {// 左右
                        var x2 = pickNode.is_connected ? idpos_map[Number(pickNode.objectName)][0]: pickNode.model_x
                        var y2 = pickNode.is_connected ? idpos_map[Number(pickNode.objectName)][1]: pickNode.model_y
                        if(mouse.x < (root.width ) / 2 && (grp_pos_mp[pickNode.group_id] === 4)) {
                            screen_pos_to_world_pos((root.width - 20) / 80,y2,pickNode)
                            if_release = false
                        } else if (mouse.x > (root.width ) / 2 && (grp_pos_mp[pickNode.group_id] === 3)){
                            screen_pos_to_world_pos((root.width - 20) / 80 - 1,y2,pickNode)
                            if_release = false
                        }
                    }
                    if(canv3.visible === true) {
                        var x_1 = pickNode.is_connected ? idpos_map[Number(pickNode.objectName)][0]: pickNode.model_x
                        var y_1 = pickNode.is_connected ? idpos_map[Number(pickNode.objectName)][1]: pickNode.model_y
                        if(mouse.y < (root.height ) / 2 && (grp_pos_mp[pickNode.group_id] === 3)) {
                            screen_pos_to_world_pos(x_1,(root.height - 20) / 80,pickNode)
                            if_release = false
                        } else if (mouse.y > (root.height ) / 2 && (grp_pos_mp[pickNode.group_id] === 1)){
                            screen_pos_to_world_pos(x_1,(root.height - 20) / 80 - 2,pickNode)
                            if_release = false
                        }
                    }
                    if(canv4.visible === true) {
                        var x_2 = pickNode.is_connected ? idpos_map[Number(pickNode.objectName)][0]: pickNode.model_x
                        var y_2 = pickNode.is_connected ? idpos_map[Number(pickNode.objectName)][1]: pickNode.model_y
                        if(mouse.y < (root.height ) / 2 && (grp_pos_mp[pickNode.group_id] === 4)) {
                            screen_pos_to_world_pos(x_2,(root.height - 20) / 80,pickNode)
                            if_release = false
                        } else if (mouse.y > (root.height ) / 2 && (grp_pos_mp[pickNode.group_id] === 2)){
                            screen_pos_to_world_pos(x_2,(root.height - 20) / 80 - 2,pickNode)
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
            ifpick = false
            event.accepted = true // 标记事件已被处理
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
                move_model(1,4)
            } else if (grp_has_pos(2) === false) {
                grp_pos_mp[grp] = 2
                move_model(2,4)
            } else if (grp_has_pos(3) === false) {
                grp_pos_mp[grp] = 3
                move_model(3,4)
            } else if (grp_has_pos(4) === false) {
                grp_pos_mp[grp] = 4
                move_model(4,4)
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
        for (var i = 0; i < plan_id.length; i++) {
            plan_id[i].visible = false
            plan_id[i].pickable = false
        }
        plan_arr.length = 0
        for (i = plan_id.length - 1; i > plan_id.length - 1 - Number(input_plan.text); i--) {
          //  console.log(plan_id[i].objectName)
            plan_id[i].visible = true
            plan_id[i].pickable = true
            plan_arr.push(plan_id[i])
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

    /*
        var map_from = control.mapFrom3DScene(node.scenePosition) // 屏幕坐标

        var pos_temp = Qt.vector3d((map_from.x + n), map_from.y, map_from.z);
        var map_to = control.mapTo3DScene(pos_temp) // 世界坐标
        sphere_node.x = map_to.x
        console.log("index ",mygrid.indexAt(2,70))*/


      //  numberDelegate.itemAt(530,30).Text="1"
     //   console.log(n,map_from.x,sphere_node.x,node.scenePosition.x,"+++++++++++++++++++++++++++++++++")
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

            // 🚀 发送位置数据给PX4
            swarm_send.caculate_pos(Number(thisnode.objectName), myIntx, myInty, 0)
        }
    }

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
                        screen_pos_to_world_pos(x1 + j * 6, y_0, form_arr[index++])
                      //  console.log("l,r",line,rows)
                    } else {// 对最后一行的特殊处理
                        var last_line_num = form_arr.length - index
                        screen_pos_to_world_pos(x1 + j * 6, y_0, form_arr[index++])

                      //  console.log("最后一排的数量,两顶点位置",last_line_num,x1,x1 + 2*(i-1)) // 1、找两定点，2、计算定点间的距离，3、按数量平分
                        if (last_line_num !== 1) {// x1是第一顶点，x1 + 2*i-1
                                var ste = 12 * (i - 1) / (last_line_num - 1)  // 修改为6倍间距
                                var step =  Math.floor(ste) // 取最大整数
                             //   console.log(ste,step)
                                for(var k = 1; k < last_line_num;k++) {
                                    if(index !== form_arr.length - 1)
                                        screen_pos_to_world_pos(x1 + k * step, y_0, form_arr[index++])
                                    else
                                       screen_pos_to_world_pos(x1 + 12*(i-1), y_0, form_arr[index++])  // 修改为6倍间距
                                }
                                break
                        }
                    }
                }
            }

            // 换行
            y_0 += 6
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

            // 计算正方形的边长（使用6的倍数）
            var side = Math.ceil(Math.sqrt(n)) * 6;
            if (side < 6) side = 6; // 最小边长为6
            var index = 0;

            // 填充四个顶点

            index += 4;

            screen_pos_to_world_pos(x_0,y_0,form_arr[0])
            screen_pos_to_world_pos(x_0,y_0 + side - 6,form_arr[1])
            screen_pos_to_world_pos(x_0 + side - 6,y_0,form_arr[2])
            screen_pos_to_world_pos(x_0 + side - 6,y_0 + side - 6,form_arr[3])

            // 填充边
            // 上边（从左到右）
            for ( i = 1; i < side/6 - 1; ++i) {
                if (index >= n) break;
                screen_pos_to_world_pos(x_0 + i * 6,y_0,form_arr[index])
                index++;
            }

            // 右边（从上到下）
            for ( i = 1; i < side/6 - 1; ++i) {
                if (index >= n) break;
                screen_pos_to_world_pos(x_0 + side - 6,y_0 + i * 6,form_arr[index])
                index++;
            }

            // 下边（从右到左）
              for (i = side/6 - 2; i >= 1; --i) {
                  if (index >= n) break;
                  screen_pos_to_world_pos(x_0 + i * 6,y_0 + side - 6,form_arr[index])
                  index++;
              }

            // 左边（从下到上）
            for (i = side/6 - 2; i >= 1; --i) {
                if (index >= n) break;
                screen_pos_to_world_pos(x_0,y_0 + i * 6,form_arr[index])
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

            console.log(size, height, x_0,y_0)
            index += 4
            screen_pos_to_world_pos(x_0 + (height - 1) * 6,y_0 ,form_arr[0])
            screen_pos_to_world_pos(x_0,y_0 + (height - 1) * 6,form_arr[1])
            screen_pos_to_world_pos(x_0 + (size - 1) * 6,y_0 + (height - 1) * 6,form_arr[2])
            screen_pos_to_world_pos(x_0 + (height - 1) * 6,y_0 + (size - 1) * 6,form_arr[3])

            // 填充边
            // 上部分（从上到下）
            for ( i = 1; i < height - 1; ++i) {
                if (index >= n) break;
                screen_pos_to_world_pos(x_0 + (height - 1 - i) * 6,y_0 + i * 6,form_arr[index])  // 左上边
                index++;
                if (index >= n) break;
                screen_pos_to_world_pos(x_0 + (height - 1 + i) * 6,y_0 + i * 6,form_arr[index]) // 右上边
                index++;
            }

            // 下部分（从上到下）
            for ( i = 1; i < height - 1; ++i) {
                if (index >= n) break;
                screen_pos_to_world_pos(x_0 + (height - 1 - i) * 6,y_0 + (size - 1 - i) * 6,form_arr[index]) // 左下边
                index++;
                if (index >= n) break;
                screen_pos_to_world_pos(x_0 + (height - 1 + i) * 6,y_0 + (size - 1 - i) * 6,form_arr[index]) // 右下边
                index++;
            }
            console.log("neibu",index,n)
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


            var radius = n * 1.5; // 进一步增大半径为飞机数量的1.5倍
            if(radius < 6) radius = 6  // 最小半径为6
            if(radius >= 18) radius = 18  // 最大半径为18
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
                x_0 += 6
            }
            screen_pos_to_world_pos(x_0 + dis, y_0, form_arr[i1])
            dis += 6
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
                y_0 += 6
            }
            screen_pos_to_world_pos(x_0, y_0 + dis, form_arr[i2])
                dis += 6
        }

        send_all_airplane_pos(input4.text,0)
    }

}
