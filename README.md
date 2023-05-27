# PathfindingTest
For the various practical problems in complex scenes of games, this responsory realized about the architecture and implementation of the whole path-finding system within the two-dimensional continuous space with several obstacles formed by polygons, which requires multi-agent to avoid collision. The main content of the path-finding system include s the following aspects:  
(1) In terms of scene analysis and visible point generating, this text presents the visible point generating algorithm based on morphological expansion on polygon-based obstacles with small calculation and convenience for partial modification, and presents another visible point generating algorithm based on two-dimensional cylinder to improve the connectivity of the visible points.  
(2) In terms of the path-finding algorithms, this text introduces a method to judge the line segments intersection based on the method of cross vector product, and puts forward the generating method of visibility graph structure cache generation, and describes the A* path-finding algorithm based on the structure in detail.  
(3) During the research on the rotational invariant bounding circles of agents, this text discusses about their collision detection algorithms. To avoid the collisions from each other, the text puts forward an improved forward collision prevention method, according to the position relationship between two circles and the detection of their going nearer to or away from each other, which improves the numerical robustness. To avoid the collisions from polygon obstacles, the text puts forward the idea to previously expand the polygons according to each units’ radius, so as to use the judgment on whether points are in those expanded polygons to approximately see the collision.  
(4) In terms of the circumambulation among the agents, the text presents circumambulating algorithms by single agents, or by multiple agents sometimes with the circumambulation ahead of time，which are within the total time complexity at O(n^2)  to guarantee the real-time performance, and are able to let the entire agents gradually close to the destination with the improvements of them.  
(5) In terms of  the combination on the circumambulating algorithms and the A* path-finding algorithm based on visible points, the text puts forward the range judgment method to let agents not too close to the visible points, so as to enhance the speed of multiple agents’ passing through the visible points, and to improve the path-finding effects. Besides, the text lets the agents find some other paths when they hit the polygon obstacles.  


对于游戏中的复杂场景出现的各种实际问题，本程序实现了在连续的二维空间内，以多边形构成的场景中，多智能体角色在不发生相互重叠的前提下的整个寻路系统的构架与实现，主要研究内容包括以下几个方面：  
(1) 在进行场景分析并生成可见点方面，提出了基于形态学膨胀的障碍附近可见点生成算法，具有计算量较小，便于局部修改的特点；并提出了另一种基于圆柱体的可见点生成算法，来生成连通性更好的可见点集合；  
(2) 在寻路过程中的相关算法方面，介绍了一种基于向量乘积的两线段相交判断的方法，并提出了可见性图结构缓存的生成方法和具体阐述了基于该结构的A Star寻路算法过程；  
(3) 用智能体中具有旋转不变性的包围圆作为研究对象，来探讨它们的碰撞检测算法，在绕行中避免智能体相互重叠方面，依据两圆之间位置关系与接近和远离的检测，提出了改进的前向预防碰撞检测方法，提升了数值健壮性；在绕行中避免与多边形障碍发生碰撞方面，提出了对场景中的各个多边形以给定的单位半径进行预膨胀的方法，从而可以用点是否在膨胀后多边形内的条件，来判断是否发生碰撞；  
(4) 对于包围圆形式的多智能体的绕行，提出了整个群体下时间复杂度为O(n^2)的，对单个智能体、多个智能体的绕行算法，以及对多智能体的提前绕行算法，并能够逐步明显地起到了总体接近目标的效果，同时保证了计算的实时性；  
(5) 在绕行算法与基于可见点的A Star寻路算法的结合方面，提出了一种对可见点的范围判断方法，让单位不至于过分接近可见点，从而提升了多个智能体通过可见点的速度，改善了寻路效果，并让单位在碰到场景中的障碍时作另外的寻路。  
