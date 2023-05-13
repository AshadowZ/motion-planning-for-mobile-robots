# hw_5

matlab部分，照着公式慢慢敲，处理完了

ros部分，闭式解也是照着公式慢慢敲，代码逻辑和matlab闭式解一样

ros部分，ooqp求解，非常的麻烦

ooqp安装可以参考这个链接：

```
https://github.com/ZJU-FAST-Lab/Fast-tracker
```

ooqp的编码也可以参考这个项目里target prediction的代码，我是参考了一段

实现上就是两个关键点

一是用不着的变量要全赋0，像这样：

```c++
const int mz = 0; // 不等式约束的个数，本作业没有不等式约束项，赋零即可
double *cupp = 0;
char *icupp = 0;
double *clow = 0;
char *iclow = 0;
const int nnzC = 0; // 不等式约束矩阵C的元素个数？，本作业没有不等式约束项，赋零即可
int *irowC = 0;
int *jcolC = 0;
double *dC = 0;
```

二是取出A矩阵的所有非零元素比较麻烦，我是用Eigen::SparseMatrix处理了

一定要记得在算总非零元素个数时把零元素全删了，因为往稀疏矩阵里插0的话它也会认为那个位置是非零的，然后数非零元素时算上

```c++
A.prune(0.0); // 删除稀疏矩阵的所有零元素
```

遍历非零元素是默认列主序的，ooqp的A必须是行主序按顺序排列的（什么坏毛病），所以稀疏矩阵A必须初始化为行主序的

```c++
Eigen::SparseMatrix<double, Eigen::RowMajor> A;
```

然后遍历

```c++
	// 取出非零元素
    int A_idx = 0;
    for (int j = 0; j < A.outerSize(); ++j) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A, j); it; ++it) {
            irowA_vec[A_idx] = it.row();
            jcolA_vec[A_idx] = it.col();
            dA_vec[A_idx] = it.value();
            A_idx++;
        }
    }
```

其他没啥，主要因为这是个很不面向对象的库，需要非常小心的慢慢赋值矩阵，体验非常糟糕，看到github上有个用ospq-eigen实现的，代码就很棒，非常美观

```
https://github.com/teamo1996/Motion-plan/blob/main/%E7%AC%AC5%E7%AB%A0%EF%BC%9A%E8%BD%A8%E8%BF%B9%E7%94%9F%E6%88%90/hw_5/ros/src/waypoint_trajectory_generator/src/trajectory_generator_waypoint.cpp
```

