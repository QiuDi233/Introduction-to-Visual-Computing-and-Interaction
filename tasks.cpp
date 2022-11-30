#include <random>

#include <spdlog/spdlog.h>

#include "Labs/1-Drawing2D/tasks.h"

using VCX::Labs::Common::ImageRGB;

template<class T>
T Max(T a, T b) {
    return a > b ? a : b;
}

template<class T>
T Min(T a, T b) {
    return a < b ? a : b;
}
int C(int m, int n) {
    if (n == 0 || m == n) return 1;
    return C(m - 1, n) + C(m - 1, n - 1);
}
namespace VCX::Labs::Drawing2D {
    /******************* 1.Image Dithering *****************/
    void DitheringThreshold(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input[{ x, y }];
                output.SetAt({ x, y }, {
                                           color.r > 0.5 ? 1 : 0,
                                           color.g > 0.5 ? 1 : 0,
                                           color.b > 0.5 ? 1 : 0,
                                       });
            }
    }

    void DitheringRandomUniform(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {

                double a = rand() / (double) (RAND_MAX); //����0-1֮��������
                a -= 0.5;                                //-0.5��0.5֮��������
                glm::vec3 color = input[{ x, y }];
                output.SetAt({ x, y }, {
                                           color.r+a > 0.5 ? 1 : 0,
                                           color.g+a > 0.5 ? 1 : 0,
                                           color.b+a > 0.5 ? 1 : 0,
                                       });
            }
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {
        // your code here:
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color1 = input[{ x, y }];
                glm::vec3 color2 = noise[{ x, y }];//0-1֮��
                output.SetAt({ x, y }, {
                                           color1.r+color2.r-0.5 > 0.5 ? 1 : 0,
                                           color1.g+color2.g-0.5 > 0.5 ? 1 : 0,
                                           color1.b+color2.b-0.5 > 0.5 ? 1 : 0,
                                       });
            }
    }

    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int order[11][3][3] = {
            {{ 0, 0, 0 },
             { 0, 0, 0 },
             { 0, 0, 0 }},//[0,0.1)

            {{ 0, 0, 0 },
             { 0, 1, 0 },
             { 0, 0, 0 }},//[0.1,0.2)

            {{ 0, 0, 0 },
             { 1, 1, 0 },
             { 0, 0, 0 }},

            {{ 0, 0, 0 },
             { 1, 1, 0 },
             { 0, 1, 0 }},

            {{ 0, 0, 0 },
             { 1, 1, 1 },
             { 0, 1, 0 }},

            {{ 0, 0, 1 },
             { 1, 1, 1 },
             { 0, 1, 0 }},

             {{ 0, 0, 1 },
             { 1, 1, 1 },
             { 1, 1, 0 }},

             {{ 1, 0, 1 },
             { 1, 1, 1 },
             { 1, 1, 0 }},

             {{ 1, 0, 1 },
             { 1, 1, 1 },
             { 1, 1, 1 }},

             {{ 1, 1, 1 },
             { 1, 1, 1 },
             { 1, 1, 1 }},//[0.9,1)

             {{ 1, 1, 1 },
             { 1, 1, 1 },
             { 1, 1, 1 }},//1
        };
        for (std::size_t x = 0; x < input.GetSizeX(); ++x) 
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
            glm::vec3 color = input[{ x, y }];
            int idx_r = (int) (10 * color.r);
            int idx_g = (int) (10 * color.g);
            int idx_b = (int) (10 * color.b);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    output.SetAt({ 3*x+i, 3*y+j }, { order[idx_r][i][j], order[idx_g][i][j], order[idx_b][i][j] });
                }
            }
            
        }
    }

    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        ImageRGB old_img = input;
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 old_color = old_img[{ x, y }];
                glm::vec3 color     = old_img[{ x, y }]; //������
                output.SetAt({ x, y }, {
                                           old_color.r > 0.5 ? 1 : 0,
                                           old_color.g > 0.5 ? 1 : 0,
                                           old_color.b > 0.5 ? 1 : 0,
                                       });
                glm::vec3 quant_error = old_color - output[{ x, y }]; //ԭ���غ������صľ�������֮������
                //�������
                if (y + 1 < input.GetSizeY()) {
                    color = old_img[{ x, y + 1 }];
                    old_img.SetAt({ x, y + 1 }, {
                                                    color.r + quant_error.r * 7 / 16,
                                                    color.g + quant_error.g * 7 / 16,
                                                    color.b + quant_error.b * 7 / 16,
                                                });
                }
                if (x + 1 < input.GetSizeX()) {
                    if (y >= 1) {
                        color = old_img[{ x + 1, y - 1 }];
                        old_img.SetAt({ x + 1, y - 1 }, {
                                                            color.r + quant_error.r * 3 / 16,
                                                            color.g + quant_error.g * 3 / 16,
                                                            color.b + quant_error.b * 3 / 16,
                                                        });
                    }
                    color = old_img[{ x + 1, y }];
                    old_img.SetAt({ x + 1, y }, {
                                                    color.r + quant_error.r * 5 / 16,
                                                    color.g + quant_error.g * 5 / 16,
                                                    color.b + quant_error.b * 5 / 16,
                                                });
                    if (y + 1 < input.GetSizeY()) {
                        color = old_img[{ x + 1, y + 1 }];
                        old_img.SetAt({ x + 1, y + 1 }, {
                                                            color.r + quant_error.r * 1 / 16,
                                                            color.g + quant_error.g * 1 / 16,
                                                            color.b + quant_error.b * 1 / 16,
                                                        });
                    }
                }
            }
    }

    /******************* 2.Image Filtering *****************/
    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input[{ x, y }];//kernel
                int       cnt   = 1;
                for (int i = -1; i <= 1; ++i) {
                    for (int j = -1; j <= 1; ++j) {
                        if (i == 0 && j == 0)
                            continue;//��ʼ����ʱ��ӹ���
                        if ((int)x + i >= 0 && x + i < input.GetSizeX()&&y+j<input.GetSizeY()&&(int)y+j>=0) {
                            color += input[{ x + i, y + j }];//����Χ�ļ�����Ȼ��ƽ��
                            cnt++;
                        }
                    }
                }
                output.SetAt({ x, y }, {
                                           color.r /cnt,
                                           color.g /cnt,
                                           color.b /cnt,
                                       });
            }
    }

    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int filter1[3][3] = {
            {1, 0, -1},
            {2, 0, -2},
            {1, 0, -1}
        };
        int filter2[3][3] = {
            { 1,  2,  1},
            { 0,  0,  0},
            {-1, -2, -1}
        };
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                if (x == 0 || y == 0 || x == input.GetSizeX() - 1 || y == input.GetSizeY() - 1) {
                    output.SetAt({ x, y }, { 0, 0, 0 });
                    continue;
                }

                glm::vec3 color1 = input[{ x, y }];
                color1 -= input[{ x, y }];//�����ʼ��Ϊ0����ֻ��������
                glm::vec3 color2 = input[{ x, y }];
                color2 -= input[{ x, y }]; //�����ʼ��Ϊ0����ֻ��������
                for (int i = -1; i <= 1; ++i) {
                    for (int j = -1; j <= 1; ++j) {
                        color1.r += input[{ x + i, y + j }].r * filter1[i + 1][j + 1];
                        color1.g += input[{ x + i, y + j }].g * filter1[i + 1][j + 1];
                        color1.b += input[{ x + i, y + j }].b * filter1[i + 1][j + 1];
                    }
                }
                for (int i = -1; i <= 1; ++i) {
                    for (int j = -1; j <= 1; ++j) {
                        color2.r += input[{ x + i, y + j }].r * filter2[i + 1][j + 1];
                        color2.g += input[{ x + i, y + j }].g * filter2[i + 1][j + 1];
                        color2.b += input[{ x + i, y + j }].b * filter2[i + 1][j + 1];
                    }
                }
                output.SetAt({ x, y }, {
                                           fabs(color1.r / 4) +fabs(color2.r / 4),
                                           fabs(color1.g / 4) +fabs(color2.g / 4),
                                           fabs(color1.b / 4) + fabs(color2.b / 4),
                                       });
            }
    }

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) {
        output             = inputBack;
        size_t      width  = inputFront.GetSizeX();
        size_t      height = inputFront.GetSizeY();
        glm::vec3 * g      = new glm::vec3[width * height];
        memset(g, 0, sizeof(glm::vec3) * width * height);
        // set boundary condition
        for (std::size_t y = 0; y < height; ++y) {
            // set boundary for (0, y), your code: g[y * width] = ?
            // set boundary for (width - 1, y), your code: g[y * width + width - 1] = ?
            //�ɺ���Ĵ�����Կ����� g��Ŀ����frontͼ�Ĳ�ֵ
            g[y * width]             = inputBack[{ (size_t) offset.x, offset.y + y }] - inputFront[{0,y}]; 
            g[y * width + width - 1] = inputBack[{ offset.x + width - 1, offset.y + y }] - inputFront[{width-1,y}];
        }
        for (std::size_t x = 0; x < width; ++x) {
            // set boundary for (x, 0), your code: g[x] = ?
            // set boundary for (x, height - 1), your code: g[(height - 1) * width + x] = ?
            g[x]                        = inputBack[{ offset.x + x, (size_t) offset.y }] - inputFront[{x,0}];
            g[(height - 1) * width + x] = inputBack[{ offset.x + x, offset.y + height - 1 }] - inputFront[{x,height-1}];
        }

        // Jacobi iteration, solve Ag = b
        for (int iter = 0; iter < 8000; ++iter) {
            for (std::size_t y = 1; y < height - 1; ++y)
                for (std::size_t x = 1; x < width - 1; ++x) {
                    g[y * width + x] = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                    g[y * width + x] = g[y * width + x] * glm::vec3(0.25);
                }
        }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                glm::vec3 color = g[y * width + x] + inputFront.GetAt({ x, y });
                output.SetAt({ x + offset.x, y + offset.y }, color);
            }
        delete[] g;
    }

    /******************* 4. Line Drawing *****************/
    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
        // your code here:
        if (p0 == p1) {//�����غ�
            canvas.SetAt({ (size_t) p0.x, (size_t) p0.y }, { color });
            return;
        }
        if (p0.x == p1.x) {//��ֱ
            size_t miny = p0.y < p1.y ? p0.y : p1.y;
            size_t maxy = p0.y < p1.y ? p1.y : p0.y;
            for (size_t y = miny; y <= maxy; ++y) {
                canvas.SetAt({ (size_t) p0.x, y }, { color });
            }
            return;
        }
        float slope = ((float) (p1.y - p0.y) /(p1.x - p0.x)); //б��
        if (slope >= 0 && slope < 1) {
            size_t x0 = p0.x < p1.x ? p0.x : p1.x;//x0<x1
            size_t x1 = p0.x < p1.x ? p1.x : p0.x;
            size_t y0 = p0.x < p1.x ? p0.y : p1.y;
            size_t y1 = p0.x < p1.x ? p1.y : p0.y;
            size_t x    = x0;
            size_t y    = y0;
            int dx   = 2 * (x1 - x0);
            int dy   = 2 * (y1 - y0);
            int dydx = dy - dx;
            int F = dy - dx / 2;
            for (x = x0; x <= x1; ++x) {
                canvas.SetAt({ x, y }, { color });
                if (F < 0)
                    F += dy;
                else {
                    y++;
                    F += dydx;
                }
            }
            return;
        }
        if (slope >= 1) {//��0��1֮��������x��y������
            size_t y0   = p0.y < p1.y ? p0.y : p1.y; // y0<y1
            size_t y1   = p0.y < p1.y ? p1.y : p0.y;
            size_t x0   = p0.y < p1.y ? p0.x : p1.x;
            size_t x1   = p0.y < p1.y ? p1.x : p0.x;
            size_t x    = x0;
            size_t y    = y0;
            int    dx   = 2 * (x1 - x0);
            int    dy   = 2 * (y1 - y0);
            int    dydx = dx - dy;
            int    F    = dx - dy / 2;
            for (y = y0; y <= y1; ++y) {
                canvas.SetAt({ x, y }, { color });
                if (F < 0)
                    F += dx;
                else {
                    x++;
                    F += dydx;
                }
            }
            return;
        }
        if (slope <0 && slope > -1) {//x���-x
            size_t x0   = p0.x < p1.x ? p1.x : p0.x; // x0>x1
            size_t x1   = p0.x < p1.x ? p0.x : p1.x;
            size_t y0   = p0.x < p1.x ? p1.y : p0.y;
            size_t y1   = p0.x < p1.x ? p0.y : p1.y;
            size_t x    = x0;
            size_t y    = y0;
            int    dx   = 2 * (-x1 + x0);
            int    dy   = 2 * (y1 - y0);
            int    dydx = dy - dx;
            int    F    = dy - dx / 2;
            for (x = x0; x >= x1; --x) {
                canvas.SetAt({ x, y }, { color });
                if (F < 0)
                    F += dy;
                else {
                    y++;
                    F += dydx;
                }
            }
            return;
        }
        if (slope <=-1) {                            //��0��1֮��������x��y������ x���-x
            size_t y0   = p0.y < p1.y ? p0.y : p1.y; // y0<y1
            size_t y1   = p0.y < p1.y ? p1.y : p0.y;
            size_t x0   = p0.y < p1.y ? p0.x : p1.x;
            size_t x1   = p0.y < p1.y ? p1.x : p0.x;
            size_t x    = x0;
            size_t y    = y0;
            int    dx   = 2 * (-x1 + x0);
            int    dy   = 2 * (y1 - y0);
            int    dydx = dx - dy;
            int    F    = dx - dy / 2;
            for (y = y0; y <= y1; ++y) {
                canvas.SetAt({ x, y }, { color });
                if (F < 0)
                    F += dx;
                else {
                    x--;
                    F += dydx;
                }
            }
            return;
        }
    }

    /******************* 5. Triangle Drawing *****************/
    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        // your code here:
        //�������˻�Ϊֱ�ߵ����
        //���������غ�
        if (p0 == p1) {
            DrawLine(canvas, color, p0, p2);
            return;
        }
        if (p0 == p2) {
            DrawLine(canvas, color, p0, p1);
            return;
        }
        if (p1 == p2) {
            DrawLine(canvas, color, p0, p1);
            return;
        }
        //���㹲��
        if ((p0.y - p1.y) * (p1.x - p2.x) == (p1.y - p2.y) * (p0.x - p1.x)) {
            DrawLine(canvas, color, p0, p1);
            DrawLine(canvas, color, p2, p1);
            DrawLine(canvas, color, p0, p2);//�����ж��ĸ�����ͷ���� �ɴ�ȫ���� �������غ�
            return;
        }
        size_t x_up = p0.y > p1.y ? (p1.y > p2.y ? p2.x : p1.x) : (p0.y > p2.y ? p2.x : p0.x);
        size_t y_up = p0.y > p1.y ? (p1.y > p2.y ? p2.y : p1.y) : (p0.y > p2.y ? p2.y : p0.y); //��������ĵ� Ҳ����y��С
        size_t x_down = p0.x > p1.x ? (p0.y > p2.y ? p0.x : p2.x) : (p1.y > p2.y ? p1.x : p2.x);
        size_t y_down = p0.y > p1.y ? (p0.y > p2.y ? p0.y : p2.y) : (p1.y > p2.y ? p1.y : p2.y); //������ĵ� y���ĵ�
        size_t x_m     = p0.x + p1.x + p2.x - x_up - x_down;//y�����м�ĵ�
        size_t y_m     = p0.y + p1.y + p2.y - y_up - y_down;
        //���м仭һ��ˮƽ���г�����
        size_t x_tmp = ((float)x_up - x_down) / ((float)y_up - y_down) * (y_m - y_up) + x_up;
        size_t xL    = x_m < x_tmp ? x_m : x_tmp;
        size_t xR    = x_m > x_tmp ? x_m : x_tmp;
        int  dxL   = (int)x_up - xL;
        int  dyL   = (int)y_up - y_m;
        int    dxR   = (int) x_up - xR;
        int    dyR   = (int) y_up - y_m;
        if (dyL == 0 || dyR == 0) {//���������������ƽ��
               //ʲô������
        } 
        else {
            float dxdyL = (float) dxL / dyL;
            float dxdyR = (float) dxR / dyR;
            //��ʼ��
            float xl = x_up;
            float xr = x_up;
            for (size_t y = y_up; y <= y_m; ++y) { // for each scanline at y
                for (size_t x = xl; x <= xr; ++x) {
                    canvas.SetAt({ x, y }, { color });
                }
                xl += dxdyL;
                xr += dxdyR;
            }
        }
        //�ٻ��°벿��
        dxL = (int) x_down - xL;
        dyL   = (int) y_down - y_m;
        dxR   = (int) x_down - xR;
        dyR   = (int) y_down - y_m;
        if (dyL == 0 || dyR == 0) {
            //������������ƽ�� ʲôҲ���� 
        } else {
            float dxdyL = (float) dxL / dyL;
            float dxdyR = (float) dxR / dyR;
            float xl    = x_down;
            float xr    = x_down;
            for (size_t y = y_down; y >= y_m; --y) { // for each scanline at y
                for (size_t x = xl; x <= xr; ++x) {
                    canvas.SetAt({ x, y }, { color });
                }
                xl -= dxdyL;
                xr -= dxdyR;
            }
        }
    }

    /******************* 6. Image Supersampling *****************/
    void Supersample(
        ImageRGB &       output,
        ImageRGB const & input,
        int              rate) {
        // your code here:
        //input 2500*2500 output 320*320 
        for (std::size_t x = 0; x < output.GetSizeX(); ++x)
            for (std::size_t y = 0; y < output.GetSizeY(); ++y) {
                glm::vec3 color = input[{ x, y }];
                //output�е�x y������input�е�125/16x 125/16y����߱ߵ�319������2492��
                //����Χ��8*8�����в��� x yΪ���Ϸ��Ǹ�
                float r = 0;
                float g = 0;
                float b = 0;
                for (int cnt = 0; cnt < rate; ++cnt) {
                    int i = rand() % 8;
                    int j = rand() % 8;
                    r += input[{ x * 125 / 16 + i, y * 125 / 16 + j }].r;
                    g += input[{ x * 125 / 16 + i, y * 125 / 16 + j }].g;
                    b += input[{ x * 125 / 16 + i, y * 125 / 16 + j }].b;
                }
                r /= rate;
                g /= rate;
                b /= rate;
                output.SetAt({ x, y }, { r, g, b });
            }
    }

    /******************* 7. Bezier Curve *****************/
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
        // your code here:
        //t��[0,1]֮����Ǹ���������
        glm::vec2 curve = { 0, 0 };
        int       n = points.size();
        for (int i = 0; i < n; ++i) {
            glm::vec2 p = points[i];
            curve.x += C(n - 1, i) * p.x * pow((1 - t), n - 1 - i) * pow(t, i);
            curve.y += C(n - 1, i) * p.y * pow((1 - t), n - 1 - i) * pow(t, i);
        }
        return curve;
    }
} // namespace VCX::Labs::Drawing2D