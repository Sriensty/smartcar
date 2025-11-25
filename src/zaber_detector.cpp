// sriensty --version=v1.0.0(目前上车版本)
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <numeric>
// #include "garage_state.hpp"
#include "search.h"

namespace zebra {

// ========== 可调参数 ==========
// // 停车库参数
// // 停车库状态：0=不在停车库，1=在停车库
// static int g_garage_state = 0;
// // 本帧事件：0=无，1=进入停车库，2=离开停车库
// static int g_garage_event = 0;
// // 进入后的“离开”抑制窗口（帧计数），>0 时禁止触发离开
// static int g_garage_cool = 0;
// static int  g_garage_anchor_x = -1; // 竖直补线的列锚点（-1 表示无）

// int g_garage_state    = 0;
// int g_garage_event    = 0;
// int g_garage_cool     = 0;
// int g_garage_anchor_x = -1;

// 停车库参数
const int required_parking_rows = 3;  // 需要连续的行数
// 添加黑白块数量上限检查
const int max_black_count = 4;  // 黑块数量上限
const int max_white_count = 4;  // 白块数量上限

struct Config {
    int   rows_from_bottom = 30; // 自底向上检测窗口（行数）
    int   min_vote_rows    = 14; // 至少多少行判“像斑马线”才成立（投票）
    int   white_margin_min = 40; // 两侧大白区（像素）阈值，超过则剔除
    int   min_black_bars   = 6;  // 至少多少个黑条
    int   min_bars         = 12;  // 至少多少个条纹（包括大白区）
    int   width_lo         = 3;  // 黑/白条宽度下限
    int   width_hi         = 11; // 黑/白条宽度上限（扩大了容忍）
    double cv_th           = 40; // 变异系数阈值（标准差/均值）
};

// 在已有参数后添加斑马线冷却参数
static int g_zebra_cool = 0;        // 斑马线冷却计数器
const int ZEBRA_COOL_FRAMES = 20;   // 斑马线冷却帧数

// 统计函数：行向 RLE（0=黑，1=白）
static inline void rle_row(const uint8_t* row, int w, std::vector<std::pair<int,int>>& runs) {
    runs.clear();
    int cur = row[0] ? 1 : 0;
    int len = 1;
    for (int i=1;i<w;++i) {
        int v = row[i] ? 1 : 0;
        if (v == cur) ++len;
        else { runs.emplace_back(cur, len); cur=v; len=1; }
    }
    runs.emplace_back(cur, len);
}

// 均值 + 变异系数（std/mean）
static inline void mean_cv(const std::vector<int>& x, double& mean, double& cv) {
    if (x.empty()) { mean=0; cv=1e9; return; }
    mean = std::accumulate(x.begin(), x.end(), 0.0) / x.size();
    if (x.size() == 1) { cv = 0; return; }
    double var=0;
    for (int v: x) { double d=v-mean; var += d*d; }
    var /= (x.size()-1);
    cv = (mean>1e-9) ? std::sqrt(var)/mean : 1e9;
}

// 停车库判定
// 新增：计算比例数组是否匹配停车库模式 [5/45, 5/45, 15/45]
static inline bool ratios_match_parking_pattern(const std::vector<double>& ratios, double tol) {
    if (ratios.size() < 3 || ratios.size() > 4) return false;

    // 目标比例
    const double small_ratio = 5.0/45.0;  // ≈ 0.111
    const double large_ratio = 15.0/45.0; // ≈ 0.333
    
    // 打印所有比例值
    printf("[PARKING] All ratios: ");
    for (double r : ratios) {
        printf("%.3f ", r);
    }
    printf("\n");

    // 匹配两种模式：[小小大] 或 [大小小]
    for (size_t i = 0; i + 2 < ratios.size(); ++i) {
        // 模式1：[小小大]
        bool pattern1 = 
            std::abs(ratios[i] - small_ratio) <= tol &&
            std::abs(ratios[i+1] - small_ratio) <= tol &&
            std::abs(ratios[i+2] - large_ratio) <= tol;

        // 模式2：[大小小]
        bool pattern2 = 
            std::abs(ratios[i] - large_ratio) <= tol &&
            std::abs(ratios[i+1] - small_ratio) <= tol &&
            std::abs(ratios[i+2] - small_ratio) <= tol;

        if (pattern1) {
            printf("[PARKING] Found pattern [小小大]: %.3f %.3f %.3f\n",
                   ratios[i], ratios[i+1], ratios[i+2]);
            return true;
        }
        if (pattern2) {
            printf("[PARKING] Found pattern [大小小]: %.3f %.3f %.3f\n",
                   ratios[i], ratios[i+1], ratios[i+2]);
            return true;
        }
    }
    
    return false;
}

// 修改后的行内停车库判断函数
// 修改后的行内停车库判断函数
static inline bool row_is_parking_by_whites(const uint8_t* row, int W, const Config& cfg, int tol = 0.05) {
    std::vector<std::pair<int,int>> runs;
    rle_row(row, W, runs);
    if ((int)runs.size() < 3) return false;

    // 剔除两侧大区块（黑或白）
    int L = 0, R = (int)runs.size() - 1;
    if (runs[L].second >= cfg.white_margin_min) ++L;
    if (runs[R].second >= cfg.white_margin_min) --R;
    if (R - L + 1 < 3) return false;

    // 检查中间区域是否存在过宽的块
    std::vector<std::pair<int,int>> valid_runs;
    for (int k = L; k <= R; ++k) {
        if (runs[k].second < cfg.white_margin_min) {
            valid_runs.push_back(runs[k]);
        } else {
            // 如果中间有过宽的块，从这里断开
            break;
        }
    }

    if (valid_runs.size() < 3) return false;

    // 必须交替
    for (size_t k = 1; k < valid_runs.size(); ++k) {
        if (valid_runs[k].first == valid_runs[k-1].first) return false;
    }

    // 收集有效区间的黑白条宽度
    std::vector<int> blacks, whites;
    int total_width = 0;
    for (const auto& run : valid_runs) {
        if (run.first == 0) {
            blacks.push_back(run.second);
        } else {
            whites.push_back(run.second);
        }
        total_width += run.second;
    }

    if (blacks.size() > max_black_count || whites.size() > max_white_count) {
        // printf("[GARAGE] 黑白块数量超限 (blacks=%zu, whites=%zu)\n", 
            //    blacks.size(), whites.size());
        return false;
    }

    if (whites.empty()) return false;

    // 计算白条比例
    std::vector<double> white_ratios;
    for (int w : whites) {
        double ratio = (double)w / total_width;
        white_ratios.push_back(ratio);
    }
    // printf("[轨道宽度]total_width: %d\n", total_width);

    // 检查是否符合停车库模式
    return ratios_match_parking_pattern(white_ratios, tol);

}

// 单行判定是否“像斑马线”
// 放宽斑马线的白条宽度和黑条宽度范围
// 放宽斑马线的白条宽度和黑条宽度范围
static inline bool looks_like_zebra_row(const uint8_t* row, int W, const Config& cfg,
                                        std::vector<int>* black_centers_out=nullptr)
{
    std::vector<std::pair<int,int>> runs;
    rle_row(row, W, runs);
    if ((int)runs.size() < cfg.min_bars) return false; // 至少有7个条纹

    // 剔除两侧大白区（白区宽度超过阈值的丢弃）
    int L = 0, R = (int)runs.size()-1;
    if (runs[L].first == 1 && runs[L].second >= cfg.white_margin_min) ++L;
    if (runs[R].first == 1 && runs[R].second >= cfg.white_margin_min) --R;
    if (R - L + 1 < cfg.min_bars-2) return false;  // 至少有5个条纹

    // 必须黑白交替，且允许一些差异
    for (int k=L+1; k<=R; ++k) {
        if (runs[k].first == runs[k-1].first){
            return false;
        } 
    }

    // 收集黑条宽度
    std::vector<int> blacks, whites;
    blacks.reserve(R-L+1);
    whites.reserve(R-L+1);
    for (int k=L; k<=R; ++k) {
        if (runs[k].first == 0) blacks.push_back(runs[k].second);
        else                   whites.push_back(runs[k].second);
    }

    // >>> 行级统计 / 打印
    printf("[ZEBRA:ROW] black_cnt=%d  white_cnt=%d\n",
           (int)blacks.size(), (int)whites.size());

    printf("  black_widths:");
    for (int w : blacks) printf(" %d", w);
    printf("\n");

    printf("  white_widths:");
    for (int w : whites) printf(" %d", w);
    printf("\n");
    // <<<

    // 允许更多样的宽度和数量
    if ((int)blacks.size() < cfg.min_black_bars) return false;

    double mb, cb, mw, cw;
    mean_cv(blacks, mb, cb);
    mean_cv(whites, mw, cw);

    // 允许的宽度范围：5-20像素
    if (mb >= cfg.width_lo && mb <= cfg.width_hi && mw >= cfg.width_lo && mw <= cfg.width_hi && cb < cfg.cv_th && cw < cfg.cv_th) {
        if (black_centers_out) {
            black_centers_out->clear();
            int x = 0;
            for (int i=0; i<L; ++i) x += runs[i].second;
            for (int k=L; k<=R; ++k) {
                int len = runs[k].second;
                if (runs[k].first == 0) {
                    int center = x + len/2;
                    black_centers_out->push_back(center);
                }
                x += len;
            }
        }
        return true;
    }
    return false;
}

// 多行投票判定 + 斜率估计
// 输入：灰度图 gray（8UC1）；输出：is_zebra，斜率 slope（列/行，图像坐标系）
// slope>0 表示往右上倾斜
static inline void detect_and_fit_zebra(const cv::Mat& gray, bool& is_zebra, double& slope,
                                        const Config& cfg = Config())
{
    // 在函数开始处添加冷却计数器的递减
    if (g_zebra_cool > 0) {
        --g_zebra_cool;
        is_zebra = false;  // 冷却期间保持false
        slope = 0.0;
        printf("[ZEBRA] 冷却中（剩余%d帧）\n", g_zebra_cool);
        return;  // 冷却期间直接返回，不进行检测
    }
    // 一帧衰减一次冷却计数
    if (g_garage_cool > 0) --g_garage_cool; 
    // 停车库识别标志
    bool seen_parking = false;
    
    CV_Assert(gray.type() == CV_8UC1);
    const int H = gray.rows, W = gray.cols;

    // 二值（OTSU）
    cv::Mat bin;
    cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // 自底向上扫描
    const int y0 = H - 1;
    const int y1 = std::max(0, H - cfg.rows_from_bottom);

    int hits = 0;
    std::vector<cv::Point2d> samples;  // (row i, black-center mean col)
    samples.reserve(cfg.rows_from_bottom);

    // 添加停车库连续行计数

    int parking_row_count = 0;            // 连续行计数器
    int last_parking_row = -1;            // 上一个检测到的行

    for (int y = y0; y >= y1; --y) {
        const uint8_t* row = bin.ptr<uint8_t>(y);

        // ① 停车库识别（基于 white_widths 模式）
        if (!seen_parking && row_is_parking_by_whites(row, W, cfg, /*tol=*/2)) {
            // 检查是否与上一行连续
            if (last_parking_row == -1 || last_parking_row == y + 1) {
                parking_row_count++;
                // printf("[GARAGE] 连续第 %d 行检测到停车库模式 (y=%d)\n", parking_row_count, y);
            } else {
                // 不连续，重置计数器
                parking_row_count = 1;
                // printf("[GARAGE] 重置连续行计数，新的停车库模式 (y=%d)\n", y);
            }
            last_parking_row = y;

            // 只有达到要求的连续行数才认为真正检测到停车库
            if (parking_row_count >= required_parking_rows) {
                seen_parking = true;

                // —— 重新做一次 RLE，取白块的起点与宽度
                std::vector<std::pair<int,int>> runs;
                rle_row(row, W, runs);

                // 剔两侧大白区（与检测时一致）
                int L = 0, R = (int)runs.size()-1;
                if (runs[L].first == 1 && runs[L].second >= cfg.white_margin_min) ++L;
                if (runs[R].first == 1 && runs[R].second >= cfg.white_margin_min) --R;

                // 收集有效区间的白块：start/len/center
                struct WSeg { int start, len, center; };
                std::vector<WSeg> whites;
                whites.reserve(R-L+1);
                int xcur = 0; for (int i=0;i<L;++i) xcur += runs[i].second;

                for (int k=L; k<=R; ++k) {
                    int len = runs[k].second;
                    if (runs[k].first == 1) {
                        WSeg s{ xcur, len, xcur + len/2 };
                        whites.push_back(s);
                    }
                    xcur += len;
                }

                // 选出"两个最小的白块"
                if ((int)whites.size() >= 2) {
                    std::nth_element(whites.begin(), whites.begin()+1, whites.end(),
                                   [](const WSeg& a, const WSeg& b){ return a.len < b.len; });
                    int c1 = whites[0].center, c2 = whites[1].center;
                    g_garage_anchor_x = (c1 + c2) / 2;
                } else {
                    g_garage_anchor_x = -1;
                }
                // std::printf("[GARAGE] y=%d 连续%d行后确认为停车库\n", y, parking_row_count);
            }
        }

        std::vector<int> centers;
        if (looks_like_zebra_row(row, W, cfg, &centers)) {
            std::cout<<"looks_like_zebra_row"<<looks_like_zebra_row<<std::endl;
            ++hits;
            // 当行用黑条中心的均值代表该行的“特征列”
            if (!centers.empty()) {
                double meanx = std::accumulate(centers.begin(), centers.end(), 0.0) / centers.size();
                samples.emplace_back((double)y, meanx);
            }
            printf("[ZEBRA:PASS] y=%d  black_cnt(centers)=%d\n", y, (int)centers.size());
        // } else {

        //     printf("[ZEBRA:FAIL] y=%d  非斑马线\n", y);
            }
        }
// 新增：保存上一帧的状态用于检测状态变化
    static bool prev_zebra = false;
    
    // 判断是否检测到斑马线
    bool new_zebra = (hits >= cfg.min_vote_rows);
    
    // 冷却逻辑
    if (g_zebra_cool > 0) {
        // 冷却期间
        --g_zebra_cool;
        is_zebra = false;
        slope = 0.0;
        // printf("[ZEBRA] 冷却中（剩余%d帧）\n", g_zebra_cool);
    } else {
        // 非冷却期间，检查是否需要触发新的冷却
        if (new_zebra && !prev_zebra) {
            // 新检测到斑马线，设置冷却
            g_zebra_cool = ZEBRA_COOL_FRAMES;
            is_zebra = true;
            printf("[ZEBRA] 检测到斑马线，设置冷却帧数=%d\n", ZEBRA_COOL_FRAMES);
        } else {
            // 正常更新状态
            is_zebra = new_zebra;
        }
        
        // 只在检测到斑马线时计算斜率
        if (is_zebra && samples.size() >= 2) {
            // 对 (row, mean_col) 做线性拟合： mean_col = k*row + b
            double Sx=0, Sy=0, Sxx=0, Sxy=0;
            for (auto &p : samples) {
                Sx  += p.x;
                Sy  += p.y;
                Sxx += p.x * p.x;
                Sxy += p.x * p.y;
            }
            const double n = (double)samples.size();
            const double denom = (n*Sxx - Sx*Sx);
            if (std::fabs(denom) > 1e-9) {
                slope = (n*Sxy - Sx*Sy) / denom; // 列/行
                // printf("[ZEBRA] 斜率计算结果: %.3f\n", slope);
            } else {
                slope = 0.0;
            }
        } else {
            slope = 0.0;
        }
    }
    
    // 更新前一帧状态
    prev_zebra = is_zebra;

    // === 停车库状态机（依赖 seen_parking） ===
    g_garage_event = 0;

    // printf("[GARAGE]初始化冷却时间%d\n", g_garage_cool);

    if (seen_parking) {
        if (g_garage_state == 0) {
            // 不在停车库 & 识别到停车库 → 进入
            if(g_garage_cool == 0){
            g_garage_state = 1;
            g_garage_event = 1;
            g_garage_cool  = 10;// 进入后，帧内不允许触发离开,重置冷却时间
            std::printf("[GARAGE] 进入停车库（state: 0→1，设置冷却=%d）\n",g_garage_cool);
            // std::cout<<"[zebra]g_garage_state:"<<g_garage_state<<std::endl;
            // 进入时若还没有 anchor_x（极少数情况），兜底用画面中线
            if (g_garage_anchor_x < 0) g_garage_anchor_x = W/2; 
            }else{
                std::printf("[GARAGE] 冷却中（%d 帧），抑制“离开”\n", g_garage_cool);
            }

        } else {
            // 在停车库 & 识别到停车库 → 离开（但若处于冷却期则抑制）
            if (g_garage_cool == 0) {
                g_garage_state = 0;
                g_garage_event = 2;
                g_garage_cool  = 10;// 离开后，帧内不允许触发进入,重置冷却时间
                // 离开后可清空锚点（可选）
                // g_garage_anchor_x = -1;
                std::printf("[GARAGE] 离开停车库（state: 1→0）\n");
                // std::cout<<"[zebra]g_garage_state:"<<g_garage_state<<std::endl;
            } else {
                // 冷却期内：不触发离开
                std::printf("[GARAGE] 冷却中（%d 帧），抑制“离开”\n", g_garage_cool);
            }
        }
    } else {
        // 本帧未识别到停车库：状态保持，事件=0
    }

}

} // namespace zebra