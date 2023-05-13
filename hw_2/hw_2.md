# hw_2

matlab版本没啥好说的，找着敲就好

ros版本，GridNodeMap\[a\]\[b\]\[c\]->id = 1 in open; -1 in close; 0 no operation

GridNodeMap是GridNodePtr ***的数据类型，维护着整个三维地图

然后就是A*的实现没有closedSet这种东西，把Map中的点id设置为-1就等于将其放在closedSet中了，代码里貌似有一处注释把id的含义交代反了，我小小修正一下

写完才发现可以参照graph_searcher.cpp写，我真是脑瘫

有时候把rviz的目标点放得很高时demo.launch的进程就崩了，报什么fault，stack limit之类的，不过正常用问题不大就忽略吧，核心逻辑已经实现了
