#include <iostream>
#include <list>
#include <map>
#include <set>
#include <unordered_set>
#include <vector>
#include <queue>
#include<math.h>
#include<algorithm>
#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

#define pi acos(-1)

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations) {
        Engine::SurfaceMesh tmp = input;
        int                 T   = numIterations;
        while (T--) {
            std::vector<glm::vec3> new_Positions;
            DCEL                   links;
            links.AddFaces(tmp.Indices);
            if (! links.IsValid()) {
                printf("links is invalid\n");
            }

            std::size_t oldv_cnt = tmp.Positions.size(); //原来的顶点的数量

            std::map<unsigned long long, int> edge_to_point;//记录 from to这条边上新产生的点
            //from to 这条边是(unsigned long long) e->From() * oldv_cnt + e->To())，类似二维数组

            //遍历原有的每一条边，在边上产生新的顶点，计算它们的位置，并在新 Mesh 中连接顶点，形成新的面
            for (DCEL::HalfEdge const * e : links.GetEdges()) {
                if (edge_to_point.count((unsigned long long) e->From() * oldv_cnt + e->To())) //这条边已经产生过顶点了
                    continue;

                glm::vec3 new_vertex = (float) 3 / 8 * tmp.Positions[e->From()]
                    + (float) 3 / 8 * tmp.Positions[e->To()]
                    + (float) 1 / 8 * tmp.Positions[e->OppositeVertex()]
                    + (float) 1 / 8 * tmp.Positions[e->PairOppositeVertex()];

                tmp.Positions.push_back(new_vertex);
                new_Positions.push_back(new_vertex);

                edge_to_point[(unsigned long long) e->From() * oldv_cnt + e->To()] = tmp.Positions.size() - 1; //新产生的这个点
                edge_to_point[(unsigned long long) e->To() * oldv_cnt + e->From()] = tmp.Positions.size() - 1;
            }
            //对于原有的每个顶点，将它们加入到新 Mesh 中，在新 Mesh 中重新计算它们的位置
            for (int i = 0; i < oldv_cnt; i++) {
                DCEL::Vertex          v        = links.GetVertex(i);
                std::vector<uint32_t> Neighbor = v.GetNeighbors();
                int                   n        = Neighbor.size();
                //权重
                float w = (5.0 / 8.0 - (3.0 / 8.0 + 1.0 / 4.0 * cos(2 * pi / n)) * (3.0 / 8.0 + 1.0 / 4.0 * cos(2 * pi / n))) / (float) n;

                tmp.Positions[i] = (1 - n * w) * tmp.Positions[i];
                for (int j = 0; j < n; j++) {
                    tmp.Positions[i] += w * tmp.Positions[Neighbor[j]];
                }
            }
            tmp.Indices.clear();//在新 Mesh 中连接顶点，形成新的面
            for (DCEL::Triangle const & f : links.GetFaces()) {
                int idx[3]; //这个面上新产生的三个点的idx
                idx[0] = edge_to_point[(unsigned long long) f.Edges(0)->From() * oldv_cnt + f.Edges(0)->To()];
                idx[1] = edge_to_point[(unsigned long long) f.Edges(1)->From() * oldv_cnt + f.Edges(1)->To()];
                idx[2] = edge_to_point[(unsigned long long) f.Edges(2)->From() * oldv_cnt + f.Edges(2)->To()];

                //按照一个面一个面的顺序放进去 原先的一个面变四个面
                tmp.Indices.push_back(idx[0]);
                tmp.Indices.push_back(idx[1]);
                tmp.Indices.push_back(idx[2]);

                tmp.Indices.push_back(*f.Indices(0));
                tmp.Indices.push_back(idx[2]);
                tmp.Indices.push_back(idx[1]);

                tmp.Indices.push_back(*f.Indices(1));
                tmp.Indices.push_back(idx[0]);
                tmp.Indices.push_back(idx[2]);

                tmp.Indices.push_back(*f.Indices(2));
                tmp.Indices.push_back(idx[1]);
                tmp.Indices.push_back(idx[0]);
            }
        }
        output = tmp;
    }




    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, const std::uint32_t numIterations) {
        DCEL links;
        links.AddFaces(input.Indices); // initialize
        if (! links.IsValid()) {
            // we check if the mesh is valid
            printf("links is in valid\n");
        }
        std::vector<int> boundary;
        int n = input.Positions.size();
        int first_side = 0;
        for (int i = 0; i < n; i++) {
            DCEL::Vertex v = links.GetVertex(i);
            if (v.IsSide()) {
                first_side = i;
                break;
               
            }
        }
        int las = -1;
        int cur = first_side;
        //要顺着找一圈 因为边界是顺着映射到uv的边界
        boundary.push_back(first_side);
        while (las == -1 || cur != first_side) {
            DCEL::Vertex v = links.GetVertex(cur);
            if (las == -1 || v.GetSideNeighbors().first == las) {
                las = cur;
                cur = v.GetSideNeighbors().second;
            } else {
                las = cur;
                cur = v.GetSideNeighbors().first;
            }
            boundary.push_back(cur);
        }
        int boundary_num = boundary.size();
        int vertex_num   = input.Positions.size();
        printf("boudary points:%d\n", boundary_num);
        std::vector<glm::vec2> uv;
        for (int i = 0; i < vertex_num; ++i) {
            uv.push_back(glm::vec2(0, 0)); // uv坐标初始化
        }
        float t = 1.0 / boundary_num;
        for (int i = 0; i < boundary_num; ++i) {
            uv[boundary[i]].x = 0.5 + 0.5 * cos(2 * pi * t * i); //需要保证在[0,1]^2之内。
            uv[boundary[i]].y = 0.5 + 0.5 * sin(2 * pi * t * i);
        }
        //迭代求解中间点的uv坐标
        int T = numIterations;
        while (T--) {
            std::vector<glm::vec2> tmp;
            for (int i = 0; i < vertex_num; ++i) {
                tmp.push_back(uv[i]);
            }
            for (int i = 0; i < vertex_num; ++i) {
                DCEL::Vertex v = links.GetVertex(i); // get vertex with index i

                if (v.IsSide()) { //检查边界点
                    continue;
                }
                uv[i]                               = glm::vec2(0, 0);
                std::vector<uint32_t> Neighbor      = v.GetNeighbors();
                int                   neighbor_size = v.GetNeighbors().size();
                float                 lamda         = 1.0 / neighbor_size; //简单起见可以使用 入ij=1/ni 的平均权重
                for (int j = 0; j < neighbor_size; j++)
                    uv[i] += lamda * tmp[Neighbor[j]];
            }
        }
        //输出的 output Mesh 的 TexCoords 应该保存每个顶点的 UV 坐标
        output = input;
        for (int i = 0; i < n; i++) {
            output.TexCoords.push_back(uv[i]);
        }
        
    }






    /******************* 3. Mesh Simplification *****************/
    struct point_pair {
        int i;
        int j;
        float cost;
        glm::vec3 v;//最优收缩点
        bool      operator<(const point_pair & p) const {
                 return cost < p.cost;
        }
    };
    std::set<std::pair<int,int>>edges;//存储有边的两个顶点
    std::set<point_pair>          pointpair;//存储点对

    glm::mat4 calKp(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3) {
        //p = [ a, b, c, d ] 其中a ^ 2 + b ^ 2 + c ^ 2 = 1
        float a = 0, b = 0, c = 0, d = 0;
        a = ((p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y));

        b = ((p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z));

        c = ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x));

        d = (0 - (a * p1.x + b * p1.y + c * p1.z));

        //正则化，使a^2+b^2+c^2=1
        float nolm = sqrt(a * a + b * b + c * c);
        a /= nolm;
        b /= nolm;
        c /= nolm;
        d /= nolm;

        // Kp=ppT 
        glm::mat4 Kp;//4*4的矩阵
        float       p[4] = { a, b, c, d };
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                Kp[i][j] = p[i] * p[j];
            }
        }
        return Kp;
    }
    float cal_cost(glm::vec4 v, glm::mat4 Q) {
        //vTQv
        glm::vec4 tmp = glm::vec4(0,0,0,0);
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                tmp[i] += v[j] * Q[j][i];
            }

        float     cost = 0;
        glm::vec4 tmp2 = glm::vec4(0, 0, 0, 0);
        for (int i = 0; i < 4; ++i) {
            cost += tmp[i] * v[i];
        }
        return cost;
    }
    float cal_best(glm::vec3 vi, glm::vec3 vj,glm::vec3 & pos, glm::mat4  Q) { //求解最优的收缩点 计算一对顶点对的代价
        glm::mat4 Q_new = Q;
        glm::mat4 Qtmp  = Q_new;
        for (int j = 0; j < 3; ++j) {
            Qtmp[3][j] = 0;
        }
        Qtmp[3][3] = 1;
        if (std::fabs(glm::determinant(Qtmp)) > 0.001) {//行列式   如果行列式为0会不可逆
            Qtmp        = glm::inverse(Qtmp);
            glm::vec4 v = glm::vec4(Qtmp[0][3], Qtmp[1][3], Qtmp[2][3], Qtmp[3][3]);
            pos.x       = v.x;
            pos.y       = v.y;
            pos.z       = v.z; //最优收缩点
            float cost  = cal_cost(v, Q_new);

            return cost;
        } else {
            pos = (vi + vj);//不可逆就取中点
            pos /= 2;
            glm::vec4 v    = glm::vec4(pos.x, pos.y, pos.z, 1);
            float     cost = cal_cost(v, Q_new);
            return cost;
        }
    }
    float distance(glm::vec3 vi, glm::vec3 vj) {
        return sqrt((vi.x - vj.x) * (vi.x - vj.x) + (vi.y - vj.y) * (vi.y - vj.y) + (vi.z - vj.z) * (vi.z - vj.z));
    }
    bool is_valid(int i, int j,glm::vec3 p1,glm::vec3 p2, float valid_pair_threshold) { //检查编号为i 和编号为j的点是否是合法点对
        if (distance(p1, p2) < valid_pair_threshold) {
            return true;
        }
        if (edges.find(std::make_pair(i, j)) != edges.end()) {
            return true;
        }
        return false;
        
    }
    std::vector<int> father;//节点的父亲
    int get_father(int x) {
        if (father[x] == x)
            return x;
        father[x] = get_father(father[x]);
        return father[x];
    }
    bool checkroot(int x) {
        return get_father(x) == x;
    }
    void Union(int u, int v) {//把u的一串挂到v上
        u     = get_father(u);
        father[u] = v;
    }
    std::vector<glm::vec3> Pos;//存储更新后的点坐标
    int                    pos_num;//更新后的点数
    void SimplifyMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, float valid_pair_threshold, float simplification_ratio) {
        /* output = input;
        return;*/
        DCEL links;
        links.AddFaces(input.Indices); // initialize
        if (! links.IsValid()) {
            printf("links is invalid\n");
        }
        //初始化set edges
        edges.clear();
        for (DCEL::HalfEdge const * e : links.GetEdges()) {
            if (e->From() > e->To())
                edges.insert(std::make_pair(e->To(), e->From()));
            else
                edges.insert(std::make_pair(e->From(), e->To()));
        }
        int vertex_num = input.Positions.size();
        Pos.clear();
        for (int i = 0; i < vertex_num; ++i) {
            Pos.push_back(input.Positions[i]);
        }
        father.clear();
        for (int i = 0; i < vertex_num; ++i) {
            father.push_back(i);
        }
        int cur = vertex_num;
        pos_num = vertex_num;
        printf("pos_num :%d\n", pos_num);
        //为每个初始顶点计算二次代价矩阵Qi
        std::vector<glm::mat4> Q(vertex_num);
        for (DCEL::Triangle const & f : links.GetFaces()) {
            glm::mat4 Kp = calKp(input.Positions[*f.Indices(0)], input.Positions[*f.Indices(1)], input.Positions[*f.Indices(2)]);
            for (int i = 0; i < 3; ++i) {
                Q[*f.Indices(i)] += Kp;
            }
        }
                              
        //按照特定规则，选择所有合法的顶点对
        pointpair.clear();
        for (int i = 0; i < input.Positions.size(); ++i) {
            for (int j = 0; j < i; ++j) {
                if (is_valid(j, i,Pos[j],Pos[i], valid_pair_threshold)) {
                    glm::vec3 v_ = glm::vec3(0, 0, 0);
                    //对于每一个顶点对 vi,vj，求解最优的收缩点v ，并计算它的代价
                    float cost = cal_best(Pos[j], Pos[i], v_, Q[i] + Q[j]);
                    pointpair.insert(point_pair(j, i, cost, v_));//编号小的放前面
                }
            }
        }
        printf("cur:%d,pos_num:%d\n", cur, pos_num);
        printf("simplification_ratio:%f\n", simplification_ratio);
        while (cur > vertex_num * simplification_ratio) {
                              
            //从顶点对中找出代价最小的那一对进行顶点合并
            if (pointpair.empty()) {
                //如果没有合法点对
                break;
            }
            point_pair best_pair = *pointpair.begin();//set中的第一个 就是代价最小的
            while (! checkroot(best_pair.i) || ! checkroot(best_pair.j) || best_pair.cost < 0) {
                pointpair.erase(pointpair.begin());
                if (pointpair.empty())
                    break;
                best_pair = *pointpair.begin();
            }
            if (pointpair.empty())
                break;

            //对选出的点进行操作
            Pos.push_back(best_pair.v);
            Q.push_back(Q[best_pair.i] + Q[best_pair.j]);

            father.push_back(pos_num);
            Union(best_pair.i, pos_num);
            Union(best_pair.j, pos_num);


             //加入新的合法点对
            for (int j = 0; j < pos_num; j++) {
                if (checkroot(j) && is_valid(j, pos_num, Pos[j], Pos[pos_num], valid_pair_threshold)) {
                    glm::vec3 v_;
                    float     cost = cal_best(Pos[j], Pos[pos_num], v_, Q[j] + Q[pos_num]);
                    pointpair.insert(point_pair(j, pos_num, cost, v_));
                }
            }

            //把原来的点连的边删掉 把新点该连的边加上
            for (int k = 0; k < pos_num; k++) {
                if (edges.find(std::make_pair(std::min(k, best_pair.i), std::max(k, best_pair.i))) != edges.end()) {
                    edges.erase(edges.find(std::make_pair(std::min(k, best_pair.i), std::max(k, best_pair.i))));
                    edges.insert(std::make_pair(k, pos_num));
            }
                if (edges.find(std::make_pair(std::min(k, best_pair.j), std::max(k, best_pair.j))) != edges.end()) {
                    edges.erase(edges.find(std::make_pair(std::min(k, best_pair.j), std::max(k, best_pair.j))));
                    edges.insert(std::make_pair(k, pos_num));
                }
            }
            
            cur--;
            pos_num++;
            printf("cur:%d,pos_num:%d\n", cur, pos_num);
        }

        for (int i = 0; i < pos_num; i++) {
            output.Positions.push_back(Pos[i]);
        }
        for (int i = 0; i < input.Indices.size(); i++) {
            output.Indices.push_back(get_father(input.Indices[i]));
        }
        printf("vertex_num : %d,pos_num %d:\n", vertex_num, pos_num);
        printf("simp end\n");
    }


















    /******************* 4. Mesh Smoothing *****************/
    float COS(glm::vec3 px, glm::vec3 py) {//求两个向量的cos角
        float dot = px.x * py.x + px.y * py.y + px.z * py.z;
        float px_len = sqrt(px.x * px.x + px.y * px.y + px.z * px.z);//|PX|
        float py_len = sqrt(py.x * py.x + py.y * py.y + py.z * py.z);//|PY|
        float c      = dot / (px_len *py_len);
        return c;
    }
    float Cotangent_Laplacian_w(glm::vec3 x, glm::vec3 y, glm::vec3 p, glm::vec3 q) {
        glm::vec3 px = p - x;
        glm::vec3 py = p - y;
        float     c1  = COS(px, py);
        float     ct1 = c1 / sqrt(1 - c1 * c1);

        glm::vec3 qx = q - x;
        glm::vec3 qy = q - y;
        float     c2  = COS(qx, qy);
        float     ct2 = c2 / sqrt(1 - c2 * c2);

        return ct1 + ct2;
    }
    void SmoothMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations, float lambda, bool useUniformWeight) {
        DCEL links;
        links.AddFaces(input.Indices); // initialize
        if (! links.IsValid()) {
            printf("links is invalid\n");
        }

        //记录 from to这条边对应的边 由点索引边
        // from to 这条边是(unsigned long long) e->From() * oldv_cnt + e->To())，类似二维数组
        int                                            vertex_num = input.Positions.size();
        std::map<unsigned long long, DCEL::HalfEdge  const *> point_to_edge; 
        
        for (DCEL::HalfEdge const * e : links.GetEdges()) {
            point_to_edge[(unsigned long long) e->From() * vertex_num + e->To()] = e;
            if (e->From() == 0 && e->To() == 1) {
                DCEL::HalfEdge const * tmp_e = e;
                printf("0-1\n");
            }
           
        }

        Engine::SurfaceMesh tmp = input;
        Engine::SurfaceMesh tmp2 = input;
        int                 T    = numIterations;
        while (T--) {
            //printf("迭代第%u次\n", numIterations - T);
            for (std::size_t i = 0; i < tmp.Positions.size(); ++i) {
                DCEL::Vertex v = links.GetVertex(i); // get vertex with index i
                //对每个顶点 vi ，计算邻居位置的加权平均
                std::vector<uint32_t> Neighbor = v.GetNeighbors();
                float                 w        = 0;
                
                float     w_sum        = 0;
                glm::vec3 Position_sum = glm::vec3(0, 0, 0);
                for (int j = 0; j < Neighbor.size(); ++j) {
                    if (useUniformWeight) {
                        //使用 Uniform Laplacian 时 wij=1
                        w = 1;
                    } else {
                        //使用 Cotangent Laplacian 时 wij=cotaij+cotbij
                        DCEL::HalfEdge const * e=point_to_edge[(unsigned long long)i * vertex_num + Neighbor[j]];
                        if (!e)//如果没有(i,Neighbor[j]这条边
                            e = point_to_edge[(unsigned long long) Neighbor[j] * vertex_num + i];
                        w                        = Cotangent_Laplacian_w(tmp.Positions[e->From()], 
                                                    tmp.Positions[e->To()], 
                                                    tmp.Positions[e->OppositeVertex()], 
                                                    tmp.Positions[e->PairOppositeVertex()]);
                        w = std::max(std::min(w, 900.0f), 0.0f);//要限制 不然w大了有洞
                        
                    }

                    w_sum += w;
                    Position_sum += w * tmp.Positions[Neighbor[j]];
                }
                glm::vec3 v_star_Position = Position_sum / w_sum;
                //更新顶点：vi = (1−入) vi + 入v∗
                glm::vec3 vi      = (1 - lambda) * tmp.Positions[i] + lambda * v_star_Position;
                tmp2.Positions[i] = vi; //把更新后的节点暂存到tmp2
            }
            for (int i = 0; i < tmp.Positions.size(); ++i) {
                tmp.Positions[i] = tmp2.Positions[i];
            }
        }
        output = tmp;
    }
    




    /******************* 5. Marching Cubes *****************/
    struct Vec3 {
        float x;
        float y;
        float z;
        bool  operator<(const Vec3 & v) const {
             if (x < v.x)
                return true;
            if (y < v.y)
                return true;
            return (z < v.z);
        }
    };

    void MarchingCubes(Engine::SurfaceMesh & output, const std::function<float(const glm::vec3 &)> & sdf, const glm::vec3 & grid_min, const float dx, const int n) {
        glm::vec3 unit[3] = { glm::vec3(1, 0, 0),
                              glm::vec3(0, 1, 0),
                              glm::vec3(0, 0, 1) };
        

        //把点坐标映射到点索引 用来检测这个点生成过没有
        std::map<Vec3, int> point_to_index;
       
        //扫描每个cube
        for (int x = 0; x < n; ++x) {
            for (int y = 0; y < n; ++y) {
                for (int z = 0; z < n; ++z) {
                    //v0的位置
                    float x0 = grid_min.x + dx * x;
                    float y0 = grid_min.y + dx * y;
                    float z0 = grid_min.z + dx * z;

                    glm::vec3 v[8];
                    uint32_t point_state=0;//二进制记录每个顶点的距离的情况（小于等于0这一位就是1）
                    for (int i = 0; i < 8; ++i) {
                        //第i个顶点的位置
                        v[i] = glm::vec3(x0 + (i & 1) * dx, y0 + (i >> 1 & 1) * dx, +z0 + (i >> 2 & 1) * dx);
                        if (sdf(v[i]) <= 0) {
                            point_state |= (1 << i);
                        }
                    }
                    //将在等值面之下的点映射到相交的边
                    uint32_t edge_state = c_EdgeStateTable[point_state];

                    int id[12];//第i条边上的点对应的index
                    for (int j = 0; j < 12; ++j) {//扫描每条边
                        if (edge_state & (1 << j)) {//这条边有点        
                            glm::vec3 start = v[0] + dx*(j & 1) * unit[((j >> 2) + 1) % 3] + dx * (j >> 1 & 1) * unit[((j >> 2) + 2) % 3];
                            glm::vec3 end   = start + unit[j >> 2]*dx;//这条边的终点

                            //计算交点坐标 P = P1 + (V – V1)·(P2 – P1)/(V2 – V1)
                            glm::vec3 new_Position = start + (0 - sdf(start)) * (end - start) / (sdf(end) - sdf(start));

                            int       pidx  = 0;
                            if (point_to_index.count(Vec3(new_Position.x,new_Position.y,new_Position.z))) {
                                //说明这个交点在别的cube里生成过了
                                pidx  = point_to_index[Vec3(new_Position.x, new_Position.y, new_Position.z)];
                                id[j] = pidx;
                            } else {
                                output.Positions.push_back(new_Position);
                                pidx = output.Positions.size() - 1;
                                point_to_index[Vec3(new_Position.x, new_Position.y, new_Position.z)] = pidx;
                                id[j]                        = pidx;
                            }
                           
                        }  
                    }
                    //查表 按顺序把点连成三角形 c_EdgeOrdsTable[point_state][j]表示这种情况下第几条边的点
                    for (int j = 0; c_EdgeOrdsTable[point_state][j] != -1; ++j) {
                        output.Indices.push_back(id[c_EdgeOrdsTable[point_state][j]]);
                    }
                }
            }
        }
        
    }
} // namespace VCX::Labs::GeometryProcessing
