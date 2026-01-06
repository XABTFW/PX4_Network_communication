#include "swarm_send.h"
#include "MultiVehicleManager.h"
#include "QGCApplication.h"
Swarm_send::Swarm_send(QObject *parent)
    : QObject{parent}
{}

void Swarm_send::set_main_airplane(int sysid, int grp_id, float x, float y, float z) {
    QMap<int, Vehicle*> mp(MultiVehicleManager::instance()->my_vehicles());
    
    // 1. 将选中的飞机设为主机
    if (mp.contains(sysid)) {
        mp[sysid]->parameterManager()->myswarm_param_send(sysid, "SWARM_SET_LEADER", FactMetaData::valueTypeInt32, 1);
    }
    
    // 2. 将同组的其他飞机设为从机
    for (auto it = group_id.begin(); it != group_id.end(); it++) {
        if (it.value() == grp_id && it.key() != sysid) {
            // 同组但不是选中的飞机 → 设为从机
            if (mp.contains(it.key())) {
                mp[it.key()]->parameterManager()->myswarm_param_send(it.key(), "SWARM_SET_LEADER", FactMetaData::valueTypeInt32, 0);
            }
        }
    }
}

void Swarm_send::store_airplane_group(int sysid, int group_id, bool flag, bool set_as_follower) {
    this->group_id[sysid] = group_id;
    // 发送分组命令
    if (flag) {
        MultiVehicleManager::instance()->my_vehicles()[sysid]->parameterManager()->myswarm_param_send(sysid, "SWARM_GROUP_ID", FactMetaData::valueTypeInt32, group_id);

        // 如果需要设为从机，同时发送 SWARM_SET_LEADER=0
        if (set_as_follower) {
            MultiVehicleManager::instance()->my_vehicles()[sysid]->parameterManager()->myswarm_param_send(sysid, "SWARM_SET_LEADER", FactMetaData::valueTypeInt32, 0);
        }
    }
}
void Swarm_send::caculate_pos(int sysid,float x,float y,float z){
    qDebug()<<__FUNCTION__<<sysid<<x<<y<<z;
    MultiVehicleManager::instance()->my_vehicles()[sysid]->parameterManager()->myswarm_param_send(sysid, "SWARM_X_OFFSET", FactMetaData::valueTypeFloat, x);
    MultiVehicleManager::instance()->my_vehicles()[sysid]->parameterManager()->myswarm_param_send(sysid, "SWARM_Y_OFFSET", FactMetaData::valueTypeFloat, y);
    MultiVehicleManager::instance()->my_vehicles()[sysid]->parameterManager()->myswarm_param_send(sysid, "SWARM_Z_OFFSET", FactMetaData::valueTypeFloat, z);
}
