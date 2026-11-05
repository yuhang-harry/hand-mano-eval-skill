# hand-mano-eval-skill

用于灵巧手 URDF 与 MANO 的几何比较及排名审计。重点是把骨架比例、零位形差、多姿态绝对误差、位移变化和指尖位置分清楚。

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

在 Codex 中使用 `$hand-mano-eval`。仓库包含原创实现和合成测试，不包含 MANO 权重、厂商模型、个人 β 数据或内部文档。模型和数据由使用者按各自许可提供；几何结果不能替代真机任务评测。
