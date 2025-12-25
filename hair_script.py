# -*- coding: utf-8 -*-
"""
aQi_HairRig_Tool - Maya毛发绑定工具
完全解耦版：UI类、算法类、Maya操作类分离
"""

import maya.cmds as cmds
import math
from math import sqrt
import traceback

# 移除无用的OpenMaya导入（避免冲突）
# import maya.api.OpenMaya as om

# ==================== 1. 纯算法类（无Maya依赖） ====================
class HairRigAlgorithm:
    @staticmethod
    def matrix_multiply(m1, m2):
        """3x3矩阵乘法：m1 × m2"""
        result = [[0]*3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                result[i][j] = sum(m1[i][k] * m2[k][j] for k in range(3))
        return result

    @staticmethod
    def normalize_vector(v):
        """向量归一化"""
        length = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
        if length > 0.0001:
            return [v[0]/length, v[1]/length, v[2]/length]
        return [0, 0, 0]

    @staticmethod
    def cross_product(v1, v2):
        """向量叉乘"""
        return [
            v1[1]*v2[2] - v1[2]*v2[1],
            v1[2]*v2[0] - v1[0]*v2[2],
            v1[0]*v2[1] - v1[1]*v2[0]
        ]

    @staticmethod
    def dot_product(v1, v2):
        """向量点乘"""
        return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]

    @staticmethod
    def z_rotation_matrix(angle_deg):
        """
        生成绕Z轴旋转指定角度的旋转矩阵（角度输入）
        :param angle_deg: 旋转角度（度）
        :return: 3x3旋转矩阵
        """
        angle_rad = math.radians(angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        # 绕Z轴旋转矩阵（右手坐标系）
        return [
            [cos_a, -sin_a, 0],
            [sin_a, cos_a,  0],
            [0,     0,      1]
        ]

    @staticmethod
    def matrix_to_euler_zyx(matrix):
        """
        旋转矩阵转换为ZYX欧拉角（Maya默认旋转顺序）
        矩阵格式: [[m00, m01, m02],
                 [m10, m11, m12],
                 [m20, m21, m22]]
        """
        # 提取矩阵元素
        m00, m01, m02 = matrix[0]
        m10, m11, m12 = matrix[1]
        m20, m21, m22 = matrix[2]

        # 计算欧拉角（ZYX顺序）
        # 注意：Maya的rotateX对应绕X轴旋转，以此类推
        rx = 0.0
        ry = 0.0
        rz = 0.0

        # 处理万向锁情况
        if abs(m20) < 0.999999:
            ry = math.asin(-m20)  # Y轴旋转
            if abs(m20) < 0.999999:
                cos_ry = math.cos(ry)
                rx = math.atan2(m21/cos_ry, m22/cos_ry)  # X轴旋转
                rz = math.atan2(m10/cos_ry, m00/cos_ry)  # Z轴旋转
            else:
                rx = 0
                rz = math.atan2(-m01, m11)
        else:
            # 万向锁情况
            ry = math.pi/2 if m20 < 0 else -math.pi/2
            rx = math.atan2(-m12, -m02)
            rz = 0

        # 弧度转角度
        rx_deg = math.degrees(rx)
        ry_deg = math.degrees(ry)
        rz_deg = math.degrees(rz)

        return [rx_deg, ry_deg, rz_deg]

    @staticmethod
    def calculate_joint_rotation(positions):
        """
        根据连续三点计算关节旋转角度
        输入: positions - 关节位置列表 [[x,y,z], ...]
        输出: 每个关节的旋转角度列表 [[rx,ry,rz], ...]
        特性: 最后一个关节在旋转矩阵层面沿自身Z轴旋转180度
        """
        if len(positions) < 3:
            print("警告: 需要至少3个点来计算旋转")
            # 返回零旋转
            return [[0, 0, 0] for _ in positions]

        rotations = []
        n = len(positions)

        # 计算中间关节的旋转（使用前后三点）
        for i in range(n):
            if i == 0:
                # 第一个关节：使用点0,1,2
                p0 = positions[0]
                p1 = positions[1]
                p2 = positions[2]
            elif i == n - 1:
                # 最后一个关节：使用点n-1, n-2, n-3
                p0 = positions[n-1]
                p1 = positions[n-2]
                p2 = positions[n-3]
            else:
                # 中间关节：使用当前点、前一个点、后一个点
                p0 = positions[i]
                p1 = positions[i+1]
                p2 = positions[i-1]

            # 1. 计算p1 = (p1 - p0)的归一化向量
            v1 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]]
            p1_norm = HairRigAlgorithm.normalize_vector(v1)

            # 2. 计算三角形法线方向
            # 向量p0->p1 和 p0->p2
            v2 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]]

            # 计算叉积得到法线
            normal = HairRigAlgorithm.cross_product(v1, v2)
            p2_norm = HairRigAlgorithm.normalize_vector(normal)

            # 如果法线太小，使用默认向上向量
            if math.sqrt(p2_norm[0]**2 + p2_norm[1]**2 + p2_norm[2]**2) < 0.001:
                p2_norm = [0, 1, 0]  # 默认Y轴向上

            # 3. 计算第三个正交向量 p3 = p1 × p2
            p3_norm = HairRigAlgorithm.cross_product(p1_norm, p2_norm)
            p3_norm = HairRigAlgorithm.normalize_vector(p3_norm)

            # 4. 构建旋转矩阵 [p1, p2, p3]^T
            # Maya中旋转矩阵是列主序：X轴(p1), Y轴(p2), Z轴(p3)
            rotation_matrix = [
                [p1_norm[0], p2_norm[0], p3_norm[0]],
                [p1_norm[1], p2_norm[1], p3_norm[1]],
                [p1_norm[2], p2_norm[2], p3_norm[2]]
            ]

            # ========== 核心修改：最后一个关节沿自身Z轴旋转180度（矩阵层面） ==========
            if i == n - 1:
                # 生成绕Z轴180度的旋转矩阵
                z_180_matrix = HairRigAlgorithm.z_rotation_matrix(180)
                # 矩阵乘法：原旋转矩阵 × Z轴180度矩阵（沿自身Z轴旋转）
                rotation_matrix = HairRigAlgorithm.matrix_multiply(rotation_matrix, z_180_matrix)
                print(f"最后一个关节：已在旋转矩阵层面沿自身Z轴旋转180度")

            # 5. 转换为欧拉角
            euler_angles = HairRigAlgorithm.matrix_to_euler_zyx(rotation_matrix)
            rotations.append(euler_angles)

            print(f"关节 {i}:")
            print(f"  位置: {p0}")
            print(f"  v1: {v1} -> p1_norm: {p1_norm}")
            print(f"  v2: {v2} -> p2_norm: {p2_norm}")
            print(f"  p3_norm: {p3_norm}")
            print(f"  旋转角度: {euler_angles}")

        return rotations

# ==================== 2. Maya操作类（仅负责Maya场景操作） ====================
class HairRigMayaOperator:
    """Maya操作类 - 仅封装Maya场景操作，依赖算法类"""

    def __init__(self):
        # 持有算法类实例
        self.algorithm = HairRigAlgorithm()

        # 状态变量
        self.mesh_name = ""
        self.joint_count = 5
        self.curve = ""
        self.vertices = []
        self.joints = []
        self.joint_positions = []
        self.joint_rotations = []

    # ---------------- 基础Maya操作 ----------------
    def get_world_position(self, node):
        """获取节点世界空间位置"""
        if cmds.objExists(node):
            return cmds.xform(node, q=True, ws=True, t=True)
        return [0, 0, 0]

    def set_world_position(self, node, position):
        """设置节点世界空间位置"""
        if cmds.objExists(node):
            cmds.xform(node, ws=True, t=position)

    def get_world_rotation(self, node):
        """获取节点世界空间旋转"""
        if cmds.objExists(node):
            return cmds.xform(node, q=True, ws=True, ro=True)
        return [0, 0, 0]

    def set_world_rotation(self, node, rotation):
        """设置节点世界空间旋转"""
        if cmds.objExists(node):
            cmds.xform(node, ws=True, ro=rotation)

    def get_or_create_group(self, group_name):
        """获取或创建空组"""
        if not cmds.objExists(group_name):
            cmds.group(empty=True, name=group_name)
        return group_name

    # ---------------- 曲线相关操作 ----------------
    def create_curve_from_selected_edges(self):
        """从选中的多边形边创建曲线（修复版）"""
        original_selection = cmds.ls(selection=True)
        if not original_selection:
            print("请先选择多边形边")
            return None

        selected_item = original_selection[0]
        self.mesh_name = selected_item.split(".")[0] if "." in selected_item else selected_item

        # 修复：捕获polyToCurve返回的曲线形状节点，找到变换节点
        self.curve = cmds.polyToCurve(
            form=2,
            degree=3,
            conformToSmoothMeshPreview=1,
            name="tmp_hair_tool_curveShape"
        )[0]
        return self.curve

    # ---------------- 骨骼相关操作 ----------------
    def get_curve_positions(self, curve_name, count):
        """
        计算曲线均匀分割点位置（0/count ~ (count-1)/count）
        修复：缩进错误+变量名冲突+参数引用错误
        """
        # 1. 输入校验（修复：将k改为count，避免未定义）
        if not cmds.objExists(curve_name):
            raise ValueError(f"曲线 {curve_name} 不存在")
        if count < 1:
            raise ValueError(f"分割数必须≥1（当前：{count}）")

        # 2. 获取曲线参数范围（修复：缩进错误）
        curve_min = cmds.getAttr(f"{curve_name}.minValue")
        curve_max = cmds.getAttr(f"{curve_name}.maxValue")
        print(f"曲线参数范围：{curve_min} ~ {curve_max}")

        # 3. 计算分割点（直接用cmds，稳定无API报错）
        split_positions = []
        # +1 补充终止点
        for i in range((count+1)):
            norm_param = i / float(count)
            actual_param = curve_min + norm_param * (curve_max - curve_min)
            pos = cmds.pointOnCurve(curve_name, parameter=actual_param)
            pos = [float(p) for p in pos]
            split_positions.append(pos)
        self.joint_positions = split_positions
        self.joint_rotations = self.algorithm.calculate_joint_rotation(self.joint_positions)
        return self.joint_positions,self.joint_rotations

    def create_joint_at_xyzabc(self, name, position,rotation):
        """在指定位置创建关节（修复：强制世界空间）"""
        cmds.select(clear=True)
        joint = cmds.joint(name=name)
        cmds.xform(joint, ws=True, t=position)  # 强制设置世界位置
        cmds.setAttr(f"{joint}.rotate", *rotation)
        cmds.makeIdentity(
            joint,
            apply=True,
            translate=True,
            rotate=True,
            scale=True,
            jointOrient=False  # 关节必须加这个参数，否则朝向会偏移
        )
        return joint

    def reverse_joint_chain(self):
        """反转骨骼链"""
        if len(self.joints) < 2:
            return

        # 断开父子关系
        for joint in self.joints:
            cmds.parent(joint, world=True)

        # 反转列表并重建父子
        self.joints.reverse()
        for i in range(1, len(self.joints)):
            if cmds.objExists(self.joints[i]) and cmds.objExists(self.joints[i-1]):
                cmds.parent(self.joints[i], self.joints[i-1])

        # 重命名关节
        for i, joint in enumerate(self.joints):
            new_name = f"{self.mesh_name}_joint{i}"
            cmds.rename(joint, new_name)
            self.joints[i] = new_name

    # ---------------- 控制器相关操作（完全重写） ----------------
    def create_controller_for_joint(self, size=1.0):
        """
        基于关节位置/旋转直接创建控制器（替换原函数）
        :param joint_idx: 关节索引（对应self.joint_positions/rotations）
        :param size: 控制器大小
        :return: 控制器信息字典
        """
        # 获取关节基础信息
        all_parent_gp_name =f"{self.joints[0]}" + "_all_parent_gp"
        for joint_idx,joint_name in enumerate(self.joints):
            print(joint_name)
            ctrl_name = f"{joint_name}_ctrl"
            ctrl_gp_name = f"{ctrl_name}_gp"
            # 获取预存的位置和旋转
            position = self.joint_positions[joint_idx]
            rotation = self.joint_rotations[joint_idx]

            # 1. 创建偏移组
            offset_group = cmds.group(empty=True, name=f"{ctrl_gp_name}")

            # 2. 创建控制器曲线
            controller = cmds.circle(
                name=ctrl_name,
                radius=size,
                normal=[1, 0, 0],
                constructionHistory=False
            )[0]

            # 5. 父化控制器到偏移组
            cmds.parent(controller, offset_group)
            # 3. 设置偏移组位置和旋转（世界空间）
            cmds.setAttr(f"{offset_group}.translate", *position)
            cmds.setAttr(f"{offset_group}.rotate", *rotation)

            # 创建总组
            if joint_idx == 0:
                all_parent_gp = cmds.group(empty=True, name=all_parent_gp_name)
                cmds.setAttr(f"{all_parent_gp}.translate", *position)
                cmds.setAttr(f"{all_parent_gp}.rotate", *rotation)

            # 6. 冻结控制器自身变换
            cmds.makeIdentity(
                controller,
                apply=True,
                translate=True,
                rotate=True,
                scale=True
            )

            # 7. 创建控制器到关节的父约束
            cmds.parentConstraint(controller, joint_name, mo=True)

            # 8. 锁定不需要的属性
            lock_attrs = ['tx', 'ty', 'tz', 'sx', 'sy', 'sz', 'v']
            for attr in lock_attrs:
                cmds.setAttr(f"{controller}.{attr}", lock=True)

            # 9. 设置控制器颜色
            shapes = cmds.listRelatives(controller, shapes=True)
            if shapes:
                cmds.setAttr(f"{shapes[0]}.overrideEnabled", 1)

        for joint_idx in range(1, len(self.joints)):  # 从1开始，跳过根控制器
            current_joint = self.joints[joint_idx]
            parent_joint = self.joints[joint_idx - 1]

            # 当前控制器的偏移组
            current_ctrl_gp = f"{current_joint}_ctrl_gp"

            # 父控制器的控制器（不是偏移组！）
            parent_ctrl = f"{parent_joint}_ctrl"

            # 检查对象是否存在
            if cmds.objExists(current_ctrl_gp) and cmds.objExists(parent_ctrl):
                # 建立父子关系：当前控制器的偏移组 → 父控制器
                cmds.parent(current_ctrl_gp, parent_ctrl)
                print(f"控制器层级: {current_ctrl_gp} → {parent_ctrl}")
            else:
                missing = []
                if not cmds.objExists(current_ctrl_gp):
                    missing.append(current_ctrl_gp)
                if not cmds.objExists(parent_ctrl):
                    missing.append(parent_ctrl)
                print(f"警告: 对象不存在 {missing}，跳过层级设置")
        cmds.parent(self.joints[0] + "_ctrl_gp", all_parent_gp_name)
        cmds.parent(self.joints[0], all_parent_gp_name)

    # ---------------- 绑定相关操作 ----------------
    def setup_skin_cluster(self):
        """创建蒙皮簇"""
        if not self.joints or not cmds.objExists(self.mesh_name):
            return False, "没有关节或网格对象"

        try:
            skin_cluster = cmds.skinCluster(
                self.joints,
                self.mesh_name,
                name=f"{self.mesh_name}_skinCluster",
                maximumInfluences=3
            )
            return True, f"蒙皮创建成功: {skin_cluster[0]}"
        except Exception as e:
            return False, f"创建蒙皮失败: {str(e)}"

    def organize_hierarchy(self, controllers):
        """组织绑定层级（仅修改控制器父子关系部分）"""
        # 骨骼层级（保留原逻辑）
        for i in range(len(self.joints) - 1, 0, -1):
            if cmds.objExists(self.joints[i]) and cmds.objExists(self.joints[i-1]):
                cmds.parent(self.joints[i], self.joints[i-1])

        # 控制器层级（重写：将子控制器偏移组父化到父控制器）
        if controllers and len(controllers) > 1:
            for i in range(1, len(controllers)):
                # 子控制器偏移组
                child_offset = controllers[i]['offset_group']
                # 父控制器（非偏移组）
                parent_ctrl = controllers[i-1]['controller']
                if cmds.objExists(child_offset) and cmds.objExists(parent_ctrl):
                    cmds.parent(child_offset, parent_ctrl)
                    print(f"控制器父子关系: {child_offset} -> {parent_ctrl}")

        # 创建组织组（保留原逻辑）
        self.create_organization_groups(controllers)
        return True, "层级组织完成"

    def create_organization_groups(self, controllers):
        """创建管理组"""
        # 关节组
        if self.joints and cmds.objExists(self.joints[0]):
            joint_group = self.get_or_create_group("hair_joints_grp")
            cmds.parent(self.joints[0], joint_group)

        # 控制器组
        if controllers and controllers[0]['offset_group']:
            ctrl_group = self.get_or_create_group("hair_controls_grp")
            cmds.parent(controllers[0]['offset_group'], ctrl_group)

        # 网格组
        if cmds.objExists(self.mesh_name):
            mesh_group = self.get_or_create_group("hair_mesh_grp")
            cmds.parent(self.mesh_name, mesh_group)

    def hide_all_elements(self):
        """隐藏所有绑定元素"""
        groups_to_hide = ["hair_joints_grp", "hair_controls_grp", "hair_mesh_grp"]
        for group_name in groups_to_hide:
            if cmds.objExists(group_name):
                cmds.setAttr(f"{group_name}.visibility", 0)
        return "所有元素已隐藏"


# ==================== 3. UI界面类（仅负责界面，持有操作类对象） ====================
class HairRigUI:
    """毛发绑定工具UI类 - 仅负责界面渲染和用户交互"""

    def __init__(self):
        # 持有Maya操作类实例
        self.operator = HairRigMayaOperator()

        # UI组件变量
        self.window_name = "hair_rig_tool_window"
        self.joint_count_field = None
        self.controller_size_field = None
        self.reverse_option = None
        self.status_label = None

    def create_ui(self):
        """创建完整UI界面"""
        # 销毁已有窗口
        if cmds.window(self.window_name, exists=True):
            cmds.deleteUI(self.window_name)

        # 创建主窗口
        window = cmds.window(
            self.window_name,
            title="毛发绑定工具",
            width=350,
            height=300,
            resizeToFitChildren=True
        )

        # 主布局
        main_layout = cmds.columnLayout(adjustableColumn=True, rowSpacing=10)

        # 标题区域
        cmds.separator(height=10, style='none')
        cmds.text(label="=== 毛发绑定工具 ===", font="boldLabelFont", height=30)
        cmds.separator(height=10, style='single')

        # 参数区域
        self._create_parameter_section()
        cmds.separator(height=10, style='single')

        # 按钮区域
        self._create_button_section()
        cmds.separator(height=10, style='single')

        # 状态区域
        self._create_status_section()

        # 显示窗口
        cmds.showWindow(window)

    def _create_parameter_section(self):
        """创建参数设置区域"""
        # 关节数量
        self.joint_count_field = cmds.intFieldGrp(
            numberOfFields=1,
            label="关节数量",
            value1=self.operator.joint_count,
            columnWidth2=[70, 50]
        )

        # 控制器大小
        self.controller_size_field = cmds.floatFieldGrp(
            numberOfFields=1,
            label="控制器大小",
            value1=1.0,
            columnWidth2=[70, 50]
        )

        # 反转骨骼链
        self.reverse_option = cmds.checkBox(
            label="反转骨骼链",
            value=False
        )

    def _create_button_section(self):
        """创建功能按钮区域"""
        # 创建骨骼按钮
        cmds.button(
            label="1. 从边创建骨骼",
            command=self.on_create_bones_clicked,
            height=35,
            backgroundColor=(0.3, 0.6, 0.9)
        )

        cmds.separator(height=5, style='none')

        # 创建完整绑定按钮
        cmds.button(
            label="2. 创建完整绑定",
            command=self.on_create_full_rig_clicked,
            height=35,
            backgroundColor=(0.2, 0.8, 0.3)
        )

        cmds.separator(height=5, style='none')

        # 隐藏按钮
        cmds.button(
            label="隐藏所有元素",
            command=self.on_hide_elements_clicked,
            height=30
        )

    def _create_status_section(self):
        """创建状态显示区域"""
        cmds.separator(height=10, style='none')
        self.status_label = cmds.text(label="状态: 等待操作", align="left")
        cmds.text(label="使用方法:", align="left")
        cmds.text(label="1. 选择多边形环边", align="left")
        cmds.text(label="2. 点击'从边创建骨骼'", align="left")
        cmds.text(label="3. 点击'创建完整绑定'", align="left")

    # ---------------- UI事件处理 ----------------
    def update_status(self, message):
        """更新状态显示"""
        cmds.text(self.status_label, edit=True, label=f"状态: {message}")

    def on_create_bones_clicked(self, *args):
        """创建骨骼按钮点击事件"""
        try:
            # 获取UI参数
            self.operator.joint_count = cmds.intFieldGrp(self.joint_count_field, q=True, value1=True)

            # 调用操作类方法创建曲线
            curve = self.operator.create_curve_from_selected_edges()
            print(curve)
            if not curve:
                cmds.warning("曲线创建失败，请先选择多边形边")
                self.update_status("曲线创建失败，请先选择多边形边")
                return

            self.update_status("曲线创建成功，开始创建骨骼...")

            # 计算曲线位置并创建骨骼
            joint_positions,joint_rotations = self.operator.get_curve_positions(curve, self.operator.joint_count)

            print(f"曲线分割点位置: {joint_positions}")
            self.operator.joints = []

            for i, position in enumerate(joint_positions):
                joint_name = f"{self.operator.mesh_name}_joint{i}"
                joint = self.operator.create_joint_at_xyzabc(joint_name, position,joint_rotations[i])
                self.operator.joints.append(joint)

            for i in range(len(self.operator.joints) - 1, 0, -1):
                child_joint = self.operator.joints[i]      # 子关节
                parent_joint = self.operator.joints[i-1]   # 父关节

                if cmds.objExists(child_joint) and cmds.objExists(parent_joint):
                    cmds.parent(child_joint, parent_joint)
                    print(f"父子关系: {child_joint} -> {parent_joint}")

            # 清理临时曲线
            if cmds.objExists(curve):
                cmds.delete(curve)

            self.update_status(f"成功创建 {len(self.operator.joints)} 个关节")
            cmds.select(self.operator.joints[-1])

        except Exception as e:
            error_msg = f"创建骨骼失败: {str(e)}"
            cmds.warning(error_msg)
            self.update_status(error_msg)
            print(traceback.format_exc())

    def on_create_full_rig_clicked(self, *args):
        """创建完整绑定按钮点击事件"""
        try:
            if not self.operator.joints:
                cmds.warning("请先创建骨骼！")
                self.update_status("请先创建骨骼")
                return

            # 获取UI参数
            controller_size = cmds.floatFieldGrp(self.controller_size_field, q=True, value1=True)
            reverse_chain = cmds.checkBox(self.reverse_option, q=True, value=True)

            # 反转骨骼链
            if reverse_chain:
                self.operator.reverse_joint_chain()
                self.update_status("骨骼链已反转")

            # 创建控制器
            self.operator.create_controller_for_joint( controller_size)

            # 创建蒙皮
            success, message = self.operator.setup_skin_cluster()
            self.update_status(message)


        except Exception as e:
            error_msg = f"创建绑定失败: {str(e)}"
            cmds.warning(error_msg)
            self.update_status(error_msg)
            print(traceback.format_exc())

    def on_hide_elements_clicked(self, *args):
        """隐藏元素按钮点击事件"""
        try:
            message = self.operator.hide_all_elements()
            self.update_status(message)
        except Exception as e:
            error_msg = f"隐藏元素失败: {str(e)}"
            cmds.warning(error_msg)
            self.update_status(error_msg)


# ==================== 主程序入口 ====================
def main():
    """启动毛发绑定工具"""
    try:
        # 创建UI实例（自动持有操作类和算法类）
        hair_rig_ui = HairRigUI()
        hair_rig_ui.create_ui()
    except Exception as e:
        cmds.error(f"工具启动失败: {str(e)}")
        print(traceback.format_exc())


if __name__ == "__main__":
    main()