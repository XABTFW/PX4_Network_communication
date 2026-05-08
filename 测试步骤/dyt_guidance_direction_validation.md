# DYT Guidance 方向定义一致性验证

## 结论

这套代码里，“方向定义是否和实际一致”的关键不在 `dyt_gimbal`，而在 `dyt_guidance` 里做的二次解释。上游驱动只是把协议里的原始角度直接解出来发布到 `dyt_target`，在 [src/drivers/dyt_gimbal/dyt_gimbal.cpp](/home/zjy/PX4DYT/tpy/src/drivers/dyt_gimbal/dyt_gimbal.cpp:486) 没有做符号修正；真正决定方向含义的是 [build_los_body()](/home/zjy/PX4DYT/tpy/src/modules/dyt_guidance/dyt_guidance.cpp:197)。

代码当前的方向链路如下：

`dyt_target` 原始角度 -> 符号/零偏修正 -> 云台系 LOS -> 机体系 -> NED

具体过程是：

- `los_x/los_y` 先乘 `DYTG_LXSIGN/DYTG_LYSIGN`
- 云台 `roll/pitch/yaw` 先乘 `DYTG_RSIGN/DYTG_PSIGN/DYTG_YSIGN`，再加 `DYTG_ROFF/POFF/YOFF`
- 然后构造 `los_gimbal = (1, tan(los_x), tan(los_y))`
- 再用云台姿态转到机体系
- 再用飞控姿态四元数把机体系转到 NED
- 最终 `los_ned / velocity_sp / yaw_sp` 都是在 NED 下输出

相关代码和消息定义：

- [src/modules/dyt_guidance/dyt_guidance.cpp](/home/zjy/PX4DYT/tpy/src/modules/dyt_guidance/dyt_guidance.cpp:197)
- [msg/DytGuidanceStatus.msg](/home/zjy/PX4DYT/tpy/msg/DytGuidanceStatus.msg:35)
- [msg/versioned/TrajectorySetpoint.msg](/home/zjy/PX4DYT/tpy/msg/versioned/TrajectorySetpoint.msg:1)
- [msg/versioned/VehicleAttitude.msg](/home/zjy/PX4DYT/tpy/msg/versioned/VehicleAttitude.msg:8)

这里有一个很重要的推断：`los_gimbal = (1, tan(los_x), tan(los_y))` 说明这个模块希望“正前方是 `+X`，右侧是 `+Y`，下方是 `+Z`”。也就是它最终想对齐到 PX4 常用的 `FRD/NED` 习惯。默认 `DYTG_LYSIGN = -1` 也说明作者大概率认为吊舱原始 `los_y` 是“向上为正”，所以要翻成 PX4 这边的“向下为正”。

## 你应该怎么验证

最有效的方法不是只看原始角，而是同时看这两个 topic：

- `listener dyt_target`
- `listener dyt_guidance_status`

建议按下面顺序做台架验证，机体尽量保持水平，先让机头朝北，方便判断：

1. 云台居中、目标在画面中心时，`dyt_guidance_status.los_ned` 应接近 `[+1, 0, 0]`。
2. 目标往画面右边移时，`los_ned.y` 应该变正，`velocity_sp.y` 也应变正。
3. 目标往画面左边移时，`los_ned.y` 应该变负。
4. 目标往画面下边移时，`los_ned.z` 应该变正；因为这是 NED，`z` 正方向是 down。
5. 目标往画面上边移时，`los_ned.z` 应该变负。
6. 机体原地偏航 90 度后，再看同一个“机头前方目标”，`los_ned` 应该从北向量转成东向量。这一步是专门验“机体系到 NED”的，不只是验相机本身。

如果你要更明确地看上游原始值和下游解释值的对应关系，可以这样理解：

- `dyt_target.los_x_rad / los_y_rad` 是驱动解出来的原始 LOS 角
- `dyt_target.gimbal_roll_rad / pitch_rad / yaw_rad` 是驱动解出来的原始云台姿态
- `dyt_guidance_status.los_ned` 是经过 `dyt_guidance` 内部符号修正、零偏修正、坐标旋转和滤波后的最终 LOS 方向
- `dyt_guidance_status.velocity_sp` 和 `yaw_sp` 是制导最终用于控制的结果

## 方向定义相关的关键参数

`dyt_guidance_params.c` 里和“方向定义”最相关、必须重点看的参数是这些：

- `DYTG_LXSIGN`：水平 LOS 角符号，见 [src/modules/dyt_guidance/dyt_guidance_params.c](/home/zjy/PX4DYT/tpy/src/modules/dyt_guidance/dyt_guidance_params.c:316)
- `DYTG_LYSIGN`：垂直 LOS 角符号，见 [src/modules/dyt_guidance/dyt_guidance_params.c](/home/zjy/PX4DYT/tpy/src/modules/dyt_guidance/dyt_guidance_params.c:325)
- `DYTG_RSIGN`：云台滚转角符号，见 [src/modules/dyt_guidance/dyt_guidance_params.c](/home/zjy/PX4DYT/tpy/src/modules/dyt_guidance/dyt_guidance_params.c:334)
- `DYTG_PSIGN`：云台俯仰角符号，见 [src/modules/dyt_guidance/dyt_guidance_params.c](/home/zjy/PX4DYT/tpy/src/modules/dyt_guidance/dyt_guidance_params.c:343)
- `DYTG_YSIGN`：云台偏航角符号，见 [src/modules/dyt_guidance/dyt_guidance_params.c](/home/zjy/PX4DYT/tpy/src/modules/dyt_guidance/dyt_guidance_params.c:352)
- `DYTG_ROFF`：云台滚转零偏，见 [src/modules/dyt_guidance/dyt_guidance_params.c](/home/zjy/PX4DYT/tpy/src/modules/dyt_guidance/dyt_guidance_params.c:362)
- `DYTG_POFF`：云台俯仰零偏，见 [src/modules/dyt_guidance/dyt_guidance_params.c](/home/zjy/PX4DYT/tpy/src/modules/dyt_guidance/dyt_guidance_params.c:372)
- `DYTG_YOFF`：云台偏航零偏，见 [src/modules/dyt_guidance/dyt_guidance_params.c](/home/zjy/PX4DYT/tpy/src/modules/dyt_guidance/dyt_guidance_params.c:382)

这些参数在 [build_los_body()](/home/zjy/PX4DYT/tpy/src/modules/dyt_guidance/dyt_guidance.cpp:197) 中统一参与方向解释。

## 出现什么现象时该看哪个参数

如果你看到这些现象，优先检查下面的参数：

- 左右反了：先查 `DYTG_LXSIGN`
- 上下反了：先查 `DYTG_LYSIGN`
- 云台转头方向和机体里理解相反：查 `DYTG_RSIGN/PSIGN/YSIGN`
- 云台明明机械居中，但 `los_ned` 总有固定偏差：查 `DYTG_ROFF/POFF/YOFF`

这几个参数里，通常最先需要确认的是：

- `DYTG_LXSIGN`
- `DYTG_LYSIGN`
- `DYTG_YOFF`

原因是实机上最常见的问题通常是：

- 图像左右定义和控制左右定义相反
- 图像上下定义和 NED 的 down 正方向不一致
- 云台安装零位和控制里假设的零位不重合

## 这些参数不改方向定义，但会让你误判

下面这些参数不直接改变“正负号定义”，但它们会让你在调试时觉得方向不对，所以也需要注意：

- `DYTG_LPF_A`
- `DYTG_PREDMAX`
- `DYTG_DLY_MS`
- `DYTG_YAWLIM`
- `DYTG_MAXYAWR`
- `DYTG_ZSCALE`
- `DYTG_MAXDZ`

它们的影响分别可以概括成：

- `DYTG_LPF_A`、`DYTG_PREDMAX`：会引入滤波或预测，方向看起来像“滞后”或“超前”
- `DYTG_DLY_MS`：主要影响拦截允许条件，不直接改变方向符号
- `DYTG_YAWLIM`、`DYTG_MAXYAWR`：会让机头跟随不彻底，看起来像偏航方向不对
- `DYTG_ZSCALE`、`DYTG_MAXDZ`：会让上下方向的响应幅度不对，但不改变正负号

## 上游驱动需要怎么理解

上游 `dyt_gimbal` 目前只是做原始解码，没有在驱动里定义“哪个方向才是最终正确方向”。相关代码在：

- [src/drivers/dyt_gimbal/dyt_gimbal.cpp](/home/zjy/PX4DYT/tpy/src/drivers/dyt_gimbal/dyt_gimbal.cpp:486)

驱动当前直接发布：

- `target.los_x_rad`
- `target.los_y_rad`
- `target.gimbal_roll_rad`
- `target.gimbal_pitch_rad`
- `target.gimbal_yaw_rad`

因此你不能只因为驱动值在变，就认定方向定义已经正确。真正要看的，是这些原始值经过 `dyt_guidance` 后，是否和 `los_ned / velocity_sp / yaw_sp` 的方向一致。

## 建议的实操结论

如果你的目标只是判断“我现在这套方向定义和实际中是不是一致”，最短路径就是：

1. 看 `listener dyt_target`
2. 看 `listener dyt_guidance_status`
3. 先验证中心、右、左、下、上五种基本方向
4. 再做一次机体偏航 90 度验证机体系到 NED 的旋转
5. 一旦左右或上下不一致，先改 `DYTG_LXSIGN/DYTG_LYSIGN`
6. 一旦中心有固定偏差，再改 `DYTG_ROFF/POFF/YOFF`

如果这几步都对上了，说明你的方向定义基本就和实际一致了。



### 下面是为了测试当云台不动时的坐标定义

 你只盯这几个字段：

  - dyt_target：los_x_rad、los_y_rad
  - dyt_guidance_status：los_ned，重点看 los_ned[1] 和 los_ned[2]

  步骤 1：目标放画面中心
  让目标在画面中心，记录一次数值。

  你希望看到的是：

  - los_x_rad 接近 0
  - los_y_rad 接近 0
  - los_ned[1] 接近 0
  - los_ned[2] 接近 0
  - los_ned[0] 比较大，接近朝前

  如果目标明明在中心，但 los_ned[1] 或 los_ned[2] 长期偏着，先怀疑零偏：

  - DYTG_YOFF
  - DYTG_POFF
  - DYTG_ROFF

  步骤 2：目标往画面右边移一点
  不要动机体，不要让云台跟着转，只让目标在画面里从中心移到右边。

  你要看两件事：

  - los_x_rad 是不是稳定朝一个方向变化
  - los_ned[1] 是不是变正

  如果目标往右时，los_ned[1] 变负，说明左右定义反了，优先查：

  - DYTG_LXSIGN

  步骤 3：目标往画面左边移一点
  同样方法，把目标从中心移到左边。

  你希望看到：

  - los_ned[1] 变负

  如果反过来变正，还是左右定义有问题，继续查：

  - DYTG_LXSIGN

  步骤 4：目标往画面下边移一点
  从中心把目标移到画面下边。

  你希望看到：

  - los_ned[2] 变正

  因为这里是 NED，z 正方向是 down。

  如果目标往下时，los_ned[2] 反而变负，说明上下定义反了，优先查：

  - DYTG_LYSIGN

  步骤 5：目标往画面上边移一点
  从中心把目标移到画面上边。

  你希望看到：

  - los_ned[2] 变负

  如果变正，还是上下定义反了，查：

  - DYTG_LYSIGN

  你只要记住这张最小对照表

  - 目标在右边：los_ned[1] 应该为正
  - 目标在左边：los_ned[1] 应该为负
  - 目标在下边：los_ned[2] 应该为正
  - 目标在上边：los_ned[2] 应该为负

  发现问题后怎么改

  - 左右反了：先改 DYTG_LXSIGN
  - 上下反了：先改 DYTG_LYSIGN
  - 中心就有固定偏差：先改 DYTG_YOFF/POFF/ROFF

  一个很重要的判断原则
  先不要管飞控有没有真的“追目标飞”，只看 dyt_guidance_status.los_ned 的方向对不对。
  因为这一步验证的是“方向定义”，不是控制效果。



### 下面是测试云台手动操作转动的情况

这里有一个假设，手动操作转云台时，在目标不动的前提下，假设目标会被自动放到导引头的中心



  先测什么
  先做“固定机体 + 固定真实目标 + 云台主动转”的测试。

  还是开两个终端：

  listener dyt_target

  listener dyt_guidance_status

  重点看：

  - dyt_target.los_x_rad
  - dyt_target.los_y_rad
  - dyt_target.gimbal_roll_rad
  - dyt_target.gimbal_pitch_rad
  - dyt_target.gimbal_yaw_rad
  - dyt_guidance_status.los_ned

  ———

  测试 1：固定目标，手动转云台偏航
  条件：

  - 机体不动
  - 选一个固定目标，比如墙上的标记
  - 让云台通过手动控制或模式切换，向左/向右转头
  - 但每次都重新把同一个目标放回画面中心附近

  你希望看到：

  - gimbal_yaw_rad 明显变化
  - los_x_rad / los_y_rad 在目标回到中心后接近 0
  - los_ned 基本不变

  这一步非常关键。
  如果同一个目标、同一个机体姿态下，只因为云台偏航角变了，los_ned 也跟着大幅转了，说明云台偏航解释
  有问题，优先查：

  - DYTG_YSIGN
  - DYTG_YOFF

  直觉上就是：

  - 云台自己转头，不应该让“世界里的目标方向”乱跳
  - 如果跳了，说明“云台角补偿”这一步有错

  ———

  测试 2：固定目标，手动转云台俯仰
  条件一样，机体不动，目标不动。

  让云台抬头/低头，但仍然去看同一个固定目标，尽量把目标重新放回中心。

  你希望看到：

  - gimbal_pitch_rad 明显变化
  - 目标回中心后，los_x_rad / los_y_rad 接近 0
  - los_ned 仍然基本不变

  如果 gimbal_pitch_rad 一变，los_ned 的上下方向就明显漂，优先查：

  - DYTG_PSIGN
  - DYTG_POFF

  ———

  测试 3：如果云台有滚转，再测 roll
  很多场景 roll 不大，优先级可以低一点。
  如果云台会主动防抖滚转，或者安装有侧倾，才做这一步。

  方法一样：

  - 机体不动
  - 目标不动
  - 让云台 roll 发生变化
  - 看同一个目标回中心后 los_ned 是否基本稳定

  如果这里有问题，查：

  - DYTG_RSIGN
  - DYTG_ROFF

  ———

  你真正要盯的现象
  这轮测试不要再用“右边/左边”那个判断法为主了。
  这轮最重要的是看“补偿后稳不稳”。

  判断标准很简单：

  - 同一个真实目标不变，los_ned 应该稳定
  - 如果云台一动，los_ned 就跟着乱变，说明 gimbal_* 的方向定义或零偏有问题

  ———

  最实用的诊断表

  - 同一目标不变，云台 yaw 变化时 los_ned 明显转偏：查 DYTG_YSIGN、DYTG_YOFF
  - 同一目标不变，云台 pitch 变化时 los_ned 明显上下漂：查 DYTG_PSIGN、DYTG_POFF
  - 同一目标不变，云台 roll 变化时 los_ned 明显侧偏：查 DYTG_RSIGN、DYTG_ROFF



  再往下一步：测自动跟踪时的云台运动
  手动转云台这轮过了，再开自动跟踪。

  这时验证的是“动态情况下整条链是否还能成立”：

  - 目标缓慢移动时，los_ned 应该平滑变化
  - 不应该出现明显反向
  - 不应该出现云台一补偿，los_ned 就跳到奇怪方向
  - velocity_sp 的方向应该和 los_ned 一致

  这一轮如果只是“抖”或者“滞后”，不一定是方向定义错了，也可能是：

  - DYTG_LPF_A
  - DYTG_PREDMAX

  这两个在影响动态观感。

  ———

  建议你按这个顺序做

  1. 固定机体，固定目标，手动转云台 yaw
  2. 固定机体，固定目标，手动转云台 pitch
  3. 如有必要，再测 roll
  4. 最后再开自动跟踪，看动态表现



### 云台自动跟踪时的测试

同一真实目标不动，云台自动补偿时，los_ned 能不能稳住
  - 真实目标慢慢移动时，los_ned 和 velocity_sp 会不会按正确方向平滑变化

  先开两个终端：

  listener dyt_target

  listener dyt_guidance_status

  重点看：

  - dyt_target：tracking_state、target_valid、los_x_rad、los_y_rad、gimbal_pitch_rad、
    gimbal_yaw_rad
  - dyt_guidance_status：los_ned、velocity_sp

  测试 1：固定目标，自动跟踪，机体不动
  做法：

  1. 选一个固定目标
  2. 让目标先不在画面中心
  3. 开自动跟踪
  4. 看云台自己把目标拉回中心

  你希望看到：

  - tracking_state = LOCKED
  - 目标被拉回中心后，los_x_rad、los_y_rad 变小，接近 0
  - gimbal_yaw_rad / gimbal_pitch_rad 明显变化
  - los_ned 不应因为云台这次补偿而乱跳
  - 最终 los_ned 应稳定指向那个固定目标方向

  这一步主要测：

  - YSIGN/YOFF
  - PSIGN/POFF
  - 必要时 RSIGN/ROFF

  测试 2：固定目标，自动跟踪，轻微转动机体
  这是最有价值的一轮。

  做法：

  1. 锁定一个固定目标
  2. 保持自动跟踪开启
  3. 人为让机体小角度偏航或俯仰
  4. 看云台是否自动反向补偿，继续盯住同一个目标

  你希望看到：

  - 机体姿态变了
  - 云台角也跟着补偿变化
  - los_x_rad、los_y_rad 仍然保持较小
  - los_ned 基本保持稳定
  - velocity_sp 不应该因为你轻微晃了一下机体就突然反向

  这一步测的是整条链路：

  - 目标 LOS
  - 云台姿态
  - 机体系
  - 机体姿态到 NED

  如果这一步失败，但静态云台测试是对的，优先怀疑：

  - DYTG_YSIGN
  - DYTG_PSIGN
  - DYTG_YOFF
  - DYTG_POFF

  测试 3：目标缓慢移动
  做法：

  1. 自动跟踪锁定目标
  2. 让目标缓慢向右、向左、向上、向下移动
  3. 看 los_ned 和 velocity_sp

  你希望看到：

  - 目标右移：los_ned[1] 朝右侧方向变化，velocity_sp.y 同方向
  - 目标左移：相反
  - 目标下移：los_ned[2] 朝正 down 方向变化
  - 整个过程应平滑，不应突然跳反

  这一步如果只是“慢半拍”或“有点抖”，不一定是方向错，也可能是：

  - DYTG_LPF_A
  - DYTG_PREDMAX

  最关键的判断标准
  自动跟踪时，不要只盯 los_x_rad/los_y_rad。
  因为自动跟踪的目标就是把它们压回接近 0。

  真正该看的反而是：

  - 云台角在变时，los_ned 是否还能正确代表真实世界方向
  - 目标真实移动时，los_ned 和 velocity_sp 是否按同一方向变化

  现象到参数的快速对应

  - 云台 yaw 一补偿，los_ned 就明显转偏：查 DYTG_YSIGN、DYTG_YOFF
  - 云台 pitch 一补偿，los_ned 就上下漂：查 DYTG_PSIGN、DYTG_POFF
  - 自动跟踪时方向大体对，但发抖或滞后：查 DYTG_LPF_A、DYTG_PREDMAX
  - velocity_sp 跟 los_ned 方向不一致：再回头查 LOS 定义和 NED 解释链路



### 如果出现发抖或者滞后的问题

  - DYTG_LPF_A：低通滤波系数，代码是
    _los_filtered = _los_filtered * alpha + los_raw * (1 - alpha)
    见 src/modules/dyt_guidance/dyt_guidance.cpp:322
  - DYTG_PREDMAX：最大预测时长，代码是
    los_body = latest.los_body + du_body * dt_pred，其中 dt_pred <= PREDMAX
    见 src/modules/dyt_guidance/dyt_guidance.cpp:299

  当前默认值是：

  - DYTG_LPF_A = 0.65，src/modules/dyt_guidance/dyt_guidance_params.c:296
  - DYTG_PREDMAX = 0.20，src/modules/dyt_guidance/dyt_guidance_params.c:307

  最重要的调参规律是：

  - LPF_A 越大：越稳、越不抖、但越滞后
  - LPF_A 越小：越灵敏、滞后越小、但越容易抖
  - PREDMAX 越大：越能“往前猜”，滞后可能变小，但更容易过冲、乱晃
  - PREDMAX 越小：越保守、更稳，但更容易显得跟慢半拍

  最实用的改法不要同时大改两个参数，按这个顺序来。

  如果主要问题是发抖
  先改 DYTG_LPF_A，每次加 0.05 到 0.10：

  - 0.65 -> 0.75
  - 还抖就 0.75 -> 0.80
  - 一般先不要超过 0.85

  如果你已经把 LPF_A 调大了还是抖，再看是不是 PREDMAX 太激进。
  这时把 DYTG_PREDMAX 往下减：

  - 0.20 -> 0.15
  - 还抖就 0.15 -> 0.10
  - 非常保守时甚至先试 0.05 或 0.00

  这类现象通常说明：

  - 原始 LOS 噪声比较大
  - 预测把噪声也一起放大了

  如果主要问题是滞后
  先不要急着大幅降 LPF_A，建议先小步试两种办法。

  办法 1：先略减 LPF_A

  - 0.65 -> 0.60
  - 还明显慢，就 0.60 -> 0.55

  办法 2：如果方向本身挺稳，只是总慢半拍，就略增 PREDMAX

  - 0.20 -> 0.25
  - 再试 0.25 -> 0.30

  一般不建议一上来把 PREDMAX 拉很大。
  因为它不是“更聪明”，只是“更敢猜”；目标轨迹不平滑时，容易变成过冲和抖动。

  最推荐的调参顺序
  如果你现在是“方向基本对，但又抖又有点慢”，按这个顺序最稳：

  1. 先固定 PREDMAX = 0.20 不动
  2. 调 LPF_A 到一个你能接受的稳定度
  3. 再小步调 PREDMAX 去补一点滞后

  也就是：

  - 先用 LPF_A 定“稳不稳”
  - 再用 PREDMAX 定“跟不跟得上”

  一个直接可试的起点
  按你说的“方向大体对，但发抖或滞后”，我建议先这样试三组：

  - 偏抖：LPF_A = 0.75, PREDMAX = 0.15
  - 折中：LPF_A = 0.70, PREDMAX = 0.20
  - 偏慢：LPF_A = 0.60, PREDMAX = 0.25

  你每次只换一组，观察：

  - dyt_guidance_status.los_ned 是否更平滑
  - velocity_sp 是否还跟得上目标移动
  - 目标停止后有没有来回晃
  - 目标加速转向时有没有明显“冲过头”

  快速判断口诀
  可以直接这么记：

  - 抖得厉害：LPF_A 加大，PREDMAX 减小
  - 慢得厉害：LPF_A 减小，PREDMAX 增大
  - 又抖又冲：优先减 PREDMAX
  - 又稳又慢：优先减 LPF_A







