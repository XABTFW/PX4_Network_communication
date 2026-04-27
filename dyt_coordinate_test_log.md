# DYT 导引头坐标定义测试记录

## 测试目的
验证云台不动时，目标在画面不同位置的坐标定义是否正确

## 关键字段
- `dyt_target`: `los_x_rad`, `los_y_rad`
- `dyt_guidance_status`: `los_ned[0]`, `los_ned[1]`, `los_ned[2]`

## 预期对照表
| 目标位置 | los_ned[1] 预期 | los_ned[2] 预期 |
|---------|----------------|----------------|
| 画面右边 | 正值 (+)        | -              |
| 画面左边 | 负值 (-)        | -              |
| 画面下边 | -              | 正值 (+)        |
| 画面上边 | -              | 负值 (-)        |
| 画面中心 | 接近 0          | 接近 0          |

---

## 步骤 1：目标放画面中心

**时间**: ___________

```
dyt_target:
  los_x_rad: _______
  los_y_rad: _______

dyt_guidance_status:
  los_ned[0]: _______  (应该比较大，朝前)
  los_ned[1]: _______  (应该接近 0)
  los_ned[2]: _______  (应该接近 0)
```

**结论**:
- [ ] 正常
- [ ] 异常：中心有固定偏差 → 检查 `DYTG_YOFF`/`DYTG_POFF`/`DYTG_ROFF`

---

## 步骤 2：目标往画面右边移

**时间**: ___________

```
dyt_target:
  los_x_rad: _______  (应该稳定朝一个方向变化)
  los_y_rad: _______

dyt_guidance_status:
  los_ned[0]: _______
  los_ned[1]: _______  (应该变正 +)
  los_ned[2]: _______
```

**结论**:
- [ ] 正常：los_ned[1] 变正
- [ ] 异常：los_ned[1] 变负 → 修改 `DYTG_LXSIGN`

---

## 步骤 3：目标往画面左边移

**时间**: ___________

```
dyt_target:
  los_x_rad: _______
  los_y_rad: _______

dyt_guidance_status:
  los_ned[0]: _______
  los_ned[1]: _______  (应该变负 -)
  los_ned[2]: _______
```

**结论**:
- [ ] 正常：los_ned[1] 变负
- [ ] 异常：los_ned[1] 变正 → 修改 `DYTG_LXSIGN`

---

## 步骤 4：目标往画面下边移

**时间**: ___________

```
dyt_target:
  los_x_rad: _______
  los_y_rad: _______

dyt_guidance_status:
  los_ned[0]: _______
  los_ned[1]: _______
  los_ned[2]: _______  (应该变正 +，因为 NED 的 z 正方向是 down)
```

**结论**:
- [ ] 正常：los_ned[2] 变正
- [ ] 异常：los_ned[2] 变负 → 修改 `DYTG_LYSIGN`

---

## 步骤 5：目标往画面上边移

**时间**: ___________

```
dyt_target:
  los_x_rad: _______
  los_y_rad: _______

dyt_guidance_status:
  los_ned[0]: _______
  los_ned[1]: _______
  los_ned[2]: _______  (应该变负 -)
```

**结论**:
- [ ] 正常：los_ned[2] 变负
- [ ] 异常：los_ned[2] 变正 → 修改 `DYTG_LYSIGN`

---

## 问题诊断与修改

### 发现的问题
- [ ] 左右反了 → 修改 `DYTG_LXSIGN` (1 ↔ -1)
- [ ] 上下反了 → 修改 `DYTG_LYSIGN` (1 ↔ -1)
- [ ] 中心有固定偏差 → 修改 `DYTG_YOFF`/`DYTG_POFF`/`DYTG_ROFF`

### 参数修改记录

| 参数名 | 修改前 | 修改后 | 修改时间 | 效果 |
|--------|--------|--------|----------|------|
| DYTG_LXSIGN | | | | |
| DYTG_LYSIGN | | | | |
| DYTG_YOFF | | | | |
| DYTG_POFF | | | | |
| DYTG_ROFF | | | | |

---

## 重要提醒

1. **测试时保持云台不动**，只移动目标在画面中的位置
2. **不要管飞控有没有真的"追目标飞"**，只看 `los_ned` 的方向对不对
3. **这一步验证的是"方向定义"**，不是控制效果
4. 每次修改参数后，记得 `param save` 保存

---

## 测试命令参考

```bash
# 查看 dyt_target
listener dyt_target

# 查看 dyt_guidance_status
listener dyt_guidance_status

# 查看当前参数值
param show DYTG_LXSIGN
param show DYTG_LYSIGN
param show DYTG_YOFF
param show DYTG_POFF
param show DYTG_ROFF

# 修改参数（示例）
param set DYTG_LXSIGN -1
param save
```
