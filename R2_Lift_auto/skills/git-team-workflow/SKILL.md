---
name: git-team-workflow
description: Use this skill whenever the user asks for Git collaboration, branch management, two-person development, weapon/meiling parallel work, resolving shared-file conflicts, or asks to apply the team's Git workflow rules. This skill is especially for the R2_Lift_auto project where one developer works on the weapon area and another works on the meiling area, with possible overlapping file edits.
---

# Git Team Workflow

这个 skill 用来执行和解释当前项目的双人 Git 协作规则。项目里默认：

- `text` 是公共稳定分支。
- `meiling` 是梅林区开发分支。
- `weapon` 是武器区开发分支。
- 用户通常负责 `meiling`。
- 用户同事通常负责 `weapon`。

所有代码注释必须使用中文。

## 工作原则

1. 不直接在 `text` 上开发业务功能。
2. 用户写梅林区代码时，优先在 `meiling` 分支。
3. 同事写武器区代码时，优先在 `weapon` 分支。
4. 每天开始开发前，先把 `text` 的最新代码合到自己的功能分支。
5. 小功能勤提交，不要攒很多天再提交。
6. 公共文件改动要小，并且改之前提醒用户需要和同事沟通。
7. 不要随意重置、回滚、覆盖他人的提交。

## 分支含义

向用户解释时使用这个简单比喻：

```text
text    = 正式本，尽量保持能编译
meiling = 用户的草稿本，用来写梅林区
weapon  = 同事的草稿本，用来写武器区
```

## 用户在梅林区开发前

先建议执行：

```powershell
# 切到梅林区分支
git switch meiling

# 获取远程最新代码
git fetch origin

# 把公共稳定分支 text 的最新代码合到当前 meiling 分支
git merge origin/text
```

如果当前仓库没有 `meiling` 分支，先检查远程分支，再决定是否创建。

不要默认强制使用 `rebase`。除非用户明确要求历史更线性，否则优先使用 `merge`，因为它对新手更直观。

## 用户完成一小段功能后

建议执行：

```powershell
# 查看当前分支和修改文件
git status

# 添加所有本次修改
git add -A

# 提交本次修改，提交信息要写清楚做了什么
git commit -m "梅林区xxx功能"

# 上传 meiling 分支
git push origin meiling
```

提交信息要简短、具体，优先使用中文，例如：

```text
梅林区状态切换
梅林区底盘速度限制
梅林区遥控输入处理
```

## 同事在武器区开发时

向用户解释同事对应执行：

```powershell
# 切到武器区分支
git switch weapon

# 获取远程最新代码
git fetch origin

# 把公共稳定分支 text 的最新代码合到当前 weapon 分支
git merge origin/text
```

完成后：

```powershell
# 查看当前修改
git status

# 添加所有本次修改
git add -A

# 提交武器区代码
git commit -m "武器区xxx功能"

# 上传 weapon 分支
git push origin weapon
```

## 交叉文件处理

如果用户和同事可能修改同一个文件，先把文件分成三类：

```text
梅林区专属文件：用户主要负责
武器区专属文件：同事主要负责
公共文件：两个人都可能改，改之前要沟通
```

常见公共文件包括：

- `main.c`
- 任务调度文件
- 状态机入口文件
- 遥控器解析文件
- CAN 通信文件
- 电机或底盘公共控制文件
- 全局配置头文件

公共文件的推荐写法是只保留入口调用，把具体逻辑放到各自模块里：

```c
// 调用梅林区任务
Merlin_Task();

// 调用武器区任务
Weapon_Task();
```

提醒用户：公共文件不要一次改一大堆。每次只围绕一个目的修改，例如“增加梅林区任务入口”或“增加武器区初始化入口”。

## 合并到 text 的建议顺序

推荐小步合并：

```text
1. 用户在 meiling 完成一个小功能
2. 用户确认能编译
3. 把 meiling 合进 text
4. 同事马上把最新 text 合进 weapon
5. 同事继续在 weapon 开发
```

避免这种做法：

```text
用户开发很多天
同事也开发很多天
两个人都改了一堆公共文件
最后才一起合并
```

这种方式冲突会很难处理。

## 冲突出现时

如果 `git merge` 出现冲突：

1. 先执行 `git status` 查看冲突文件。
2. 不要直接删除冲突内容。
3. 打开冲突文件，找到 `<<<<<<<`、`=======`、`>>>>>>>`。
4. 对每个冲突块判断：保留用户逻辑、保留同事逻辑，还是合并两边逻辑。
5. 修改后重新编译或运行项目验证。
6. 再执行 `git add` 和 `git commit` 完成合并。

解释冲突时使用简单说法：

```text
Git 不是说代码坏了，而是不知道同一个位置该保留谁的内容，需要人判断。
```

## 执行命令前的检查

如果用户让你实际操作 Git：

1. 先执行 `git status`。
2. 看清楚当前分支。
3. 看清楚有没有未提交修改。
4. 不要覆盖用户未提交代码。
5. 如果需要切分支但当前有修改，先说明风险，再建议提交或 stash。

禁止在没有用户明确同意时执行：

```powershell
git reset --hard
git checkout -- .
git clean -fd
```

## 回答风格

面向用户解释时，不要说太多术语。优先使用：

- 正式本
- 草稿本
- 上传
- 同步
- 合并
- 冲突

每次给命令时都写中文注释，方便用户理解。

