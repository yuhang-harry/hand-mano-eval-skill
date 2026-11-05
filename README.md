# hand-mano-eval-skill

用于灵巧手 URDF 与 MANO 的几何比较及排名审计。重点是把骨架比例、零位形差、多姿态绝对误差、位移变化和指尖位置分清楚。

URDF没有天然的β。先按声明的特征匹配或拟合MANO手型β和整手尺度α，再冻结它们，标定参考关节角与配准，并逐姿态求机器人关节角q。β的数值大小、绝对位置误差或单个子指标都不能直接作为整体能力排名。

- [SKILL.md](SKILL.md)：执行流程与判断边界。
- [计算方法](references/method.md)：公式、标定、统计与敏感性。
- [输入与命令](references/inputs.md)：可复用的 FK、Jacobian、MANO LBS 和指标脚本。
- [Sharpa W01 审计案例](references/audit-example.md)：为什么几何低分不能直接解释为产品动作能力差。

安装到 Codex：

```sh
git clone https://github.com/yuhang-harry/hand-mano-eval-skill.git ~/.codex/skills/hand-mano-eval
```

在能访问目标数据的 Python 环境中安装 `numpy`、`scipy`，随后运行：

```sh
python scripts/metrics.py self-check
python scripts/validate_core.py
```

上述命令运行合成自检；用户输入的指标计算命令见[输入与命令](references/inputs.md)。仓库提供FK、MANO适配器、评分模块及通用标定／逐姿态IK程序；使用者提供已核对的点定义、目标姿态和模型资产，不包含原实验私有数据。

在 Codex 中使用 `$hand-mano-eval`。仓库包含原创实现和合成测试，不包含 MANO 权重、厂商模型、个人 β 数据或内部文档。模型和数据由使用者按各自许可提供；几何结果不能替代真机任务评测。


完整手指比较使用20个手指点，标定与测试采用同一点集。腕／机械安装代理另列。`scripts/landmarks.py`严格解析型号自身的mesh并写入显式点偏移；`scripts/evaluate.py`执行标定与限位IK。
