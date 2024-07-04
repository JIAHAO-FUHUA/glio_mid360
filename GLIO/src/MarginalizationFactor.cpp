#include "factors/MarginalizationFactor.h"

//多线程计算雅可比矩阵和残差的构造，最终生成系统矩阵A和向量b
/*
获取参数块索引和大小：循环遍历每个残差块的参数块，获取每个参数块的索引和大小。
调整参数块大小：将大小为4的参数块调整为3（可能是四元数）。
计算雅可比矩阵：提取雅可比矩阵的相关部分。
构造矩阵 A 和向量 b：计算并累加系统矩阵 A 的块和向量 b 的值。
*/
void *ThreadsConstructA(void *threadsstruct) {
    ThreadsStruct *p = ((ThreadsStruct *) threadsstruct);
    for (auto it : p->sub_factors) {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++) {
            int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
            int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
            if (size_i == 4)
                size_i = 3;
            Eigen::MatrixXd jacobian_i = it->jacobians[i].rightCols(size_i);
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++) {
                int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
                if (size_j == 4)
                    size_j = 3;
                Eigen::MatrixXd jacobian_j = it->jacobians[j].rightCols(size_j);
                if (i == j)
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                else {
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    return threadsstruct;
}

//用于评估残差块和雅可比矩阵，并在必要时应用损失函数对残差进行调整。
/****************
初始化残差和雅可比矩阵：为每个参数块分配存储残差和雅可比矩阵的空间。
评估残差和雅可比矩阵：调用 cost_function 的 Evaluate 方法，计算残差和雅可比矩阵。
应用损失函数：如果存在损失函数，则计算损失函数值并调整残差和雅可比矩阵。调整的步骤包括计算残差的平方范数，调用损失函数进行评估，并根据评估结果调整残差和雅可比矩阵。
*****************/

void ResidualBlockInfo::Evaluate() {
    residuals.resize(cost_function->num_residuals());

    std::vector<int> block_sizes = cost_function->parameter_block_sizes();
    raw_jacobians = new double *[block_sizes.size()];
    jacobians.resize(block_sizes.size());

    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
        jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
        raw_jacobians[i] = jacobians[i].data();
    }
    cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);

    if (loss_function) {
        double residual_scaling_, alpha_sq_norm_;

        double sq_norm, rho[3];

        sq_norm = residuals.squaredNorm();
        loss_function->Evaluate(sq_norm, rho);

        double sqrt_rho1_ = sqrt(rho[1]);

        if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
            residual_scaling_ = sqrt_rho1_;
            alpha_sq_norm_ = 0.0;
        } else {
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_ = alpha / sq_norm;
        }

        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
            jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));

        }

        residuals *= residual_scaling_;
    }
}

//释放 MarginalizationInfo 对象中的动态分配内存，以防止内存泄漏。
MarginalizationInfo::~MarginalizationInfo() {
    for (auto it = parameter_block_data.begin(); it != parameter_block_data.end(); ++it)
        delete it->second;  // 释放参数块数据的内存

    for (int i = 0; i < (int) factors.size(); i++) {

        delete[] factors[i]->raw_jacobians;  // 释放雅可比矩阵的内存

        delete factors[i]->cost_function;  // 释放代价函数的内存

        delete factors[i];  // 释放残差块信息的内存
    }
}


//用于将残差块信息添加到 MarginalizationInfo 对象中。
void MarginalizationInfo::AddResidualBlockInfo(ResidualBlockInfo *residual_block_info) {
    factors.emplace_back(residual_block_info);  // 添加残差块信息到 factors 列表

    std::vector<double *> &parameter_blocks = residual_block_info->parameter_blocks;
    std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();

    //遍历 residual_block_info 的参数块，存储每个参数块的大小。
    for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++) {
        double *addr = parameter_blocks[i];
        int size = parameter_block_sizes[i];
        parameter_block_size[reinterpret_cast<long>(addr)] = size;   // 存储参数块大小
    }

    if(residual_block_info->drop_set.size() == 0) return;

    for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); i++) {
        double *addr = parameter_blocks[residual_block_info->drop_set[i]];
        parameter_block_idx[reinterpret_cast<long>(addr)] = 0;   // 标记将要丢弃的参数块
    }
}

//边缘化之前对残差块进行预处理。
//遍历 factors，调用每个残差块的 Evaluate 方法进行评估。
//遍历评估后的参数块，复制并存储参数块数据到 parameter_block_data 中。
void MarginalizationInfo::PreMarginalize() {
    for (auto it : factors) {
        it->Evaluate();  // 评估残差块

        std::vector<int> block_sizes = it->cost_function->parameter_block_sizes();
        for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
            long addr = reinterpret_cast<long>(it->parameter_blocks[i]);
            int size = block_sizes[i];
            if (parameter_block_data.find(addr) == parameter_block_data.end()) {
                double *data = new double[size];
                memcpy(data, it->parameter_blocks[i], sizeof(double) * size);  // 复制参数块数据
                parameter_block_data[addr] = data;  // 存储参数块数据
            }
        }
    }
}

//根据输入的大小返回局部大小。如果大小为4，则返回3（通常用于四元数的情况）。
int MarginalizationInfo::LocalSize(int size) const {
    return size == 4 ? 3 : size;
}

//功能执行边缘化过程，计算出系统矩阵 A 和向量 b，并对其进行求解和更新。
/************
计算参数块的索引和大小：
初始化矩阵 A 和向量 b：
多线程计算 A 和 b：
计算边缘化矩阵和向量：
更新线性化雅可比矩阵和残差：
*****************/
void MarginalizationInfo::Marginalize() {
    int pos = 0;
    //遍历 parameter_block_idx，为每个参数块分配位置 pos，并根据 LocalSize 计算局部大小。
    for (auto &it : parameter_block_idx) {
        it.second = pos;
        pos += LocalSize(parameter_block_size[it.first]);
    }

    //计算边缘化参数块的总大小 m
    m = pos;

    //遍历 parameter_block_size，为未分配位置的参数块分配位置 pos，并计算总大小 n。
    for (const auto &it : parameter_block_size) {
        if (parameter_block_idx.find(it.first) == parameter_block_idx.end()) {
            parameter_block_idx[it.first] = pos;
            pos += LocalSize(it.second);
        }
    }

    n = pos - m;

    //初始化矩阵 A 和向量 b
    Eigen::MatrixXd A(pos, pos);
    Eigen::VectorXd b(pos);
    A.setZero();
    b.setZero();

    //多线程计算 A 和 b
    //创建线程结构 ThreadsStruct 并将残差块分配给各线程。
    pthread_t tids[NUM_THREADS];
    ThreadsStruct threadsstruct[NUM_THREADS];
    int i = 0;
    for (auto it : factors) {
        threadsstruct[i].sub_factors.push_back(it);
        i++;
        i = i % NUM_THREADS;
    }


    for (int i = 0; i < NUM_THREADS; i++) {

         //为每个线程创建零矩阵和零向量，并将参数块大小和索引传递给线程结构。
        threadsstruct[i].A = Eigen::MatrixXd::Zero(pos, pos);
        threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
        threadsstruct[i].parameter_block_size = parameter_block_size;
        threadsstruct[i].parameter_block_idx = parameter_block_idx;
        //创建线程并执行 ThreadsConstructA 计算每个子块的 A 和 b
        int ret = pthread_create(&tids[i], NULL, ThreadsConstructA, (void *) &(threadsstruct[i]));
        if (ret != 0) {
            ROS_DEBUG("pthread_create error");
            ROS_BREAK();
        }
    }

    //合并所有线程计算的结果，得到全局矩阵 A 和向量 b
    for (int i = NUM_THREADS - 1; i >= 0; i--) {
        pthread_join(tids[i], NULL);
        A += threadsstruct[i].A;
        b += threadsstruct[i].b;
    }

    //计算边缘化矩阵和向量
    //提取子矩阵 Amm，并计算其对称正定矩阵分解。
    Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);

    //计算 Amm 的逆矩阵 Amm_inv
    Eigen::MatrixXd Amm_inv = saes.eigenvectors()
            * Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal()
            * saes.eigenvectors().transpose();
    //提取子向量 bmm，子矩阵 Amr，子矩阵 Arm 和子矩阵Arr。
    Eigen::VectorXd bmm = b.segment(0, m);
    Eigen::MatrixXd Amr = A.block(0, m, m, n);
    Eigen::MatrixXd Arm = A.block(m, 0, n, m);
    Eigen::MatrixXd Arr = A.block(m, m, n, n);
    Eigen::VectorXd brr = b.segment(m, n);
    //计算新的矩阵 A 和向量 b
    A = Arr - Arm * Amm_inv * Amr;
    b = brr - Arm * Amm_inv * bmm;

    //对新的矩阵 A 进行对称正定矩阵分解
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);

    //计算新的矩阵 A 的特征值和特征向量，并计算平方根和逆平方根。
    Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
    Eigen::VectorXd
            S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

    Eigen::VectorXd S_sqrt = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

    //计算线性化后的雅可比矩阵 linearized_jacobians 和线性化后的残差 linearized_residuals。
    linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
}

//参数块的地址映射中获取要保留的参数块，并返回这些参数块的地址。
std::vector<double *> MarginalizationInfo::GetParameterBlocks(std::unordered_map<long, double *> &addr_shift) {
    std::vector<double *> keep_block_addr;
    keep_block_size.clear();
    keep_block_idx.clear();
    keep_block_data.clear();

    //遍历 parameter_block_idx，找到索引大于或等于 m 的参数块。
    //记录这些参数块的大小、索引和数据，并将其地址存储在 keep_block_addr 中。
    for (const auto &it : parameter_block_idx) {
        if (it.second >= m) {
            keep_block_size.push_back(parameter_block_size[it.first]);
            keep_block_idx.push_back(parameter_block_idx[it.first]);
            keep_block_data.push_back(parameter_block_data[it.first]);
            keep_block_addr.push_back(addr_shift[it.first]);
        }
    }
    //计算 keep_block_size 向量中所有元素的总和，存储在 sum_block_size 中。
    sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);
    //返回要保留的参数块地址
    return keep_block_addr;
}

//初始化边缘化因子，设置参数块的大小和残差数量
//将 _marginalization_info 赋值给成员变量 marginalization_info。
MarginalizationFactor::MarginalizationFactor(MarginalizationInfo *_marginalization_info) : marginalization_info(
                                                                                               _marginalization_info) {
    int cnt = 0;
    //遍历 keep_block_size 向量，将每个元素的值添加到 parameter_block_sizes 向量中，并累加到 cnt 中。
    for (auto it : marginalization_info->keep_block_size) {
        mutable_parameter_block_sizes()->push_back(it);
        cnt += it;
    }
    //设置残差的数量为 n。
    set_num_residuals(marginalization_info->n);
};

//用于评估残差和雅可比矩阵。
bool MarginalizationFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    int n = marginalization_info->n;
    int m = marginalization_info->m;
    //计算参数块的差异向量 dx
    //遍历每个保留的参数块，计算当前参数值和初始参数值的差异。如果参数块的大小为4（四元数），则使用四元数差异计算方法。
    Eigen::VectorXd dx(n);
    for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++) {
        int size = marginalization_info->keep_block_size[i];
        int idx = marginalization_info->keep_block_idx[i] - m;
        Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
        Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size);
        if (size != 4) {
            dx.segment(idx, size) = x - x0;
        }
        else {
            dx.segment<3>(idx) = 2.0 * (Eigen::Quaterniond(x0(0), x0(1), x0(2), x0(3)).inverse()
                                        * Eigen::Quaterniond(x(0), x(1), x(2), x(3))).normalized().vec();
            if ((Eigen::Quaterniond(x0(0), x0(1), x0(2), x0(3)).inverse() * Eigen::Quaterniond(x(0), x(1), x(2), x(3))).w()
                    < 0) {
                dx.segment<3>(idx) = -2.0 * (Eigen::Quaterniond(x0(0), x0(1), x0(2), x0(3)).inverse()
                                             * Eigen::Quaterniond(x(0), x(1), x(2), x(3))).normalized().vec();
            }
        }
    }

    //使用线性化的残差和雅可比矩阵，计算残差向量。
    Eigen::Map<Eigen::VectorXd>(residuals, n) =
            marginalization_info->linearized_residuals + marginalization_info->linearized_jacobians * dx;

    //计算雅可比矩阵
    if (jacobians) {

        for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++) {
            if (jacobians[i]) {

                int size = marginalization_info->keep_block_size[i], local_size = marginalization_info->LocalSize(size);
                int idx = marginalization_info->keep_block_idx[i] - m;
                //
                Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
                Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size);
                //
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
                        jacobian(jacobians[i], n, size);
                jacobian.setZero();
                if(size != 4)
                    jacobian.rightCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx, local_size);
                else {
                    if ((Eigen::Quaterniond(x0(0), x0(1), x0(2), x0(3)).inverse() * Eigen::Quaterniond(x(0), x(1), x(2), x(3))).w() >= 0)
                        jacobian.rightCols(size) = 2.0 * marginalization_info->linearized_jacobians.middleCols(idx, local_size) *
                                Qleft(Eigen::Quaterniond(x0(0), x0(1), x0(2), x0(3)).inverse()).bottomRightCorner<3, 4>();
                    else
                        jacobian.rightCols(size) = -2.0 * marginalization_info->linearized_jacobians.middleCols(idx, local_size) *
                                Qleft(Eigen::Quaterniond(x0(0), x0(1), x0(2), x0(3)).inverse()).bottomRightCorner<3, 4>();
                }
            }
        }
    }
    return true;
}
