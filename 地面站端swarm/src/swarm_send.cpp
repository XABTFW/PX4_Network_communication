#include "swarm_send.h"
#include "MultiVehicleManager.h"
#include "QGCApplication.h"
Swarm_send::Swarm_send(QObject *parent)
    : QObject{parent}
{}

void Swarm_send::set_main_airplane(int sysid, int grp_id, float x,float y,float z) { // 需要给飞控发送自定义消息 设置主机
  //  qDebug()<<sysid<<x<<y<<z<<__FUNCTION__;

    QMap<int , Vehicle*> mp(MultiVehicleManager::instance()->my_vehicles());
    auto it = mp.begin();
    for (;it != mp.end(); it++) {
        if (it.key() == sysid) {
            it.value()->parameterManager()->myswarm_param_send(sysid, "SWARM_SET_LEADER", FactMetaData::valueTypeInt32, 1);
        } else {
             // 将同组的其他飞机设为从机
            auto itt = group_id.begin();
            for (; itt != group_id.end(); itt++) {
                if (grp_id == itt.value() && sysid != itt.key()) {
                    mp[itt.key()]->parameterManager()->myswarm_param_send(itt.key(), "SWARM_SET_LEADER", FactMetaData::valueTypeInt32, 0);
                }
            }
        }
    }
}

void Swarm_send::store_airplane_group(int sysid, int group_id, bool flag) {
    this->group_id[sysid] = group_id;
    // 发送分组命令
    if (flag) {
        MultiVehicleManager::instance()->my_vehicles()[sysid]->parameterManager()->myswarm_param_send(sysid, "SWARM_GROUP_ID", FactMetaData::valueTypeInt32, group_id);
    }
}
void Swarm_send::caculate_pos(int sysid,float x,float y,float z){
    MultiVehicleManager::instance()->my_vehicles()[sysid]->parameterManager()->myswarm_param_send(sysid, "SWARM_X_OFFSET", FactMetaData::valueTypeFloat, x);
    MultiVehicleManager::instance()->my_vehicles()[sysid]->parameterManager()->myswarm_param_send(sysid, "SWARM_Y_OFFSET", FactMetaData::valueTypeFloat, y);
    MultiVehicleManager::instance()->my_vehicles()[sysid]->parameterManager()->myswarm_param_send(sysid, "SWARM_Z_OFFSET", FactMetaData::valueTypeFloat, z);
}
