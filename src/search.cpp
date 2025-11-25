// sriensty --version==1.0（稳定上车）
#include "search.h"
#include <iostream>
#include "math.h"
#if(KEIL)
#include "zf_common_typedef.h"
#endif
using namespace std;

#include <cstring>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>

// #include "zaber_detector.cpp"


int IMAGE_MIDDLE = 130;
/********************************************变量声明***************************************************/
uint8        g_VideoImageData[MAX_VIDEO_LINE][MAX_VIDEO_POINT];  //图像原始数据，g_VideoImageData[m][n]代表图像的从上数第m行，从左数第n点，0为最黑，255为最白
//==============================图像数据================================//
_POINT  g_CenterPosition[MAX_POINT_NUM];
_POINT  g_LeftEdge[MAX_POINT_NUM], g_RightEdge[MAX_POINT_NUM];
int          g_CenterNum;
int          g_LeftEdgeNum, g_RightEdgeNum;
//==============================图像处理用变量==========================//
uint16       g_Start[11], g_End[11], g_SEnum;
uint8        g_SearchFlag = 1;
int          g_Cover;
uint8        g_CoverIndex[11], g_CoverNum;
int              g_connect_edge_count = 0;
int              g_connect_edge_start = 0;
bool         possible_cross = false;
//==============================控制用变量==============================//
int                     g_DirectionControl = MID_VIDEO_POINT;
int                     g_FormerDirectionControl = MID_VIDEO_POINT;
int                     g_DirectionControlWhole = 0;
int                     g_DirectionControlLine = 0;
//==============================函数==============================//
void findQuadraticCoefficients(double y1, double x1, double y2, double x2, double y3, double x3, double &a, double &b, double &c);
// void findQuadraticCoefficients(double y1, double x1, double y2, double x2, double y3, double x3);
float Slope_Calculate(uint8 begin, uint8 end, _POINT* border); //最小二乘法拟合直线

//==============================新添的变量==============================//
// ==== 通用小工具 ====
#ifndef ABS
#define ABS(a)                ((a) >= 0 ? (a) : -(a))
#endif
#ifndef MIN
#define MIN(a,b)              ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b)              ((a) > (b) ? (a) : (b))
#endif
#ifndef CLAMP
#define CLAMP(v,lo,hi)        ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))
#endif
#define ROUNDI(x)             ((int)((x) >= 0 ? ((x)+0.5f) : ((x)-0.5f)))

// // 停车库参数
// namespace zebra{
// extern int g_garage_state;
// extern int g_garage_event;
// extern int g_garage_cool;
// extern int g_garage_anchor_x;   // ← 新增：zaber_detector 里算的锚点
// }
int g_garage_state    = 0;
int g_garage_event    = 0;
int g_garage_cool     = 0;
int g_garage_anchor_x = -1;

// ==== 可运行时改的参数====
//十字参数
static const int   XR_BOTTOM_ROWS = 97;  //搜索行数
static const int   XR_EDGE_TOL    = 1;   //贴边容差（像素）
static const int   XR_DY_TOL      = 2;
static const int   XR_MIN_RUN     = 6;
static const int   XR_GAP_ALLOW   = 1;
static const int   XR_ROW_TOL     = 18;   // 左右贴边段“中点行”的相近阈值（行）
static const int   XR_EXTEND_N    = 18;   // 延长行数
static const int   XR_COOL_FRM    = 12;   // 冷却（行）
static const float XR_BLEND       = 0.85f;

const int SKIP_TOP_ROWS = 20;    // 跳过顶部的行数
const int SKIP_BOTTOM_ROWS = 36; // 跳过底部的行数

typedef struct {
    /* 贴边（底部窗口） */
    int   bottom_rows;      // 仅在底部这几行内检查贴边（从0行往上数）
    int   stick_min_run;    // 连续行数阈值
    int   stick_gap_allow;  // 允许空洞
    int   stick_edge_tol;   // 与边界容差（像素）
    int   stick_dy_tol;     // “竖直一致性”：相邻行y差的容差
    int   stick_latch_len;  // 侧向锁定长度（行），防左右抖

    /* 防突变（限幅，不冻结） */
    int   k_win;            // 小窗口回归的窗口大小（行）
    float k_delta_max;      // 每步允许的斜率增量上限
    float ema_alpha;        // 最后 EMA 系数(0..1)；0 关闭
} Cfg;

static volatile Cfg g_cfg = {
    /* bottom_rows      */ 77,
    /* stick_min_run    */ 40,
    /* stick_gap_allow  */ 1,
    /* stick_edge_tol   */ 0,
    /* stick_dy_tol     */ 1,
    /* stick_latch_len  */ 24,

    /* k_win            */ 5,
    /* k_delta_max      */ 0.25f,
    /* ema_alpha        */ 0.40f
};

// 导入斑马线参数
extern bool   g_has_zebra;
extern double g_zebra_k;

const int ZEBRA_BOTTOM_ROWS = 60;     // 从底部向上检测的行数
const int ZEBRA_START_ROW = 5;       // 从底部跳过的行数（避免最底部的噪声）
const int ZEBRA_MIN_STRIPES = 8;      // 最少需要的黑条数量
const int ZEBRA_WIDTH_TOL = 8;        // 条纹宽度容差(像素)
const int ZEBRA_MIN_WIDTH = 7;        // 最小条纹宽度
const int ZEBRA_MAX_WIDTH = 15;       // 最大条纹宽度
const int ZEBRA_CONFIRM_ROWS = 5;     // 需要连续确认的行数

/* ---------- 帧内状态 ---------- */
static int  s_stick_side  = 0;   // -1:左贴  0:无  +1:右贴
static int  s_stick_latch = 0;   // 还剩几行强制按该侧处理
static bool s_locked_rows[MAX_VIDEO_LINE]; // 贴边段锁定

/* ---------- 小工具 ---------- */
#ifndef LANE_DRAW_LOCK_MASK_DEFINED
#define LANE_DRAW_LOCK_MASK_DEFINED

static bool s_draw_row[MAX_VIDEO_LINE];// —— 行渲染开关：false 表示这一行“蓝线不再绘制/更新”
static inline void clear_locked(){ for(int i=0;i<MAX_VIDEO_LINE;++i) s_locked_rows[i]=false; }
static inline void clear_draw_mask(){ for(int i=0;i<MAX_VIDEO_LINE;++i) s_draw_row[i]=true; }


static int first_border_hit_row(int tol=0){
    for(int i=0;i<g_CenterNum;++i){
        if (g_CenterPosition[i].y <= 0+tol) return i;
        if (g_CenterPosition[i].y >= MAX_VIDEO_POINT-1-tol) return i;
    }
    return -1;
}
#endif

// 改进后的撞边检测和停止补线函数
static void stop_after_border_hit() {
    // 从下向上遍历所有中心点
    for (int i = 0; i < g_CenterNum; ++i) {
        // 检查是否触及左边界或右边界
        if (g_CenterPosition[i].y <= 0 || g_CenterPosition[i].y >= MAX_VIDEO_POINT-1) {
            // 找到撞边点，将该点以上的所有行标记为不绘制并锁定
            for (int k = i; k < g_CenterNum; ++k) {  
                s_draw_row[k] = false;
                s_locked_rows[k] = true;
            }
            // 找到第一个撞边点后立即退出
            break;
        }
    }
}




static inline int bottom_center_y() {
    if (g_LeftEdgeNum > 0 && g_RightEdgeNum > 0)
        return (g_LeftEdge[0].y + g_RightEdge[0].y) / 2;
    return MAX_VIDEO_POINT / 2;
}

static float lr_slope(int i0, int i1) {
    if (g_CenterNum <= 0) return 0.f;
    int lo = CLAMP(i0, 0, g_CenterNum-1);
    int hi = CLAMP(i1, 0, g_CenterNum-1);
    if (hi <= lo) return 0.f;
    double Sx=0,Sy=0,Sxx=0,Sxy=0; int n=0;
    for (int i=lo;i<=hi;++i) {
        double x=g_CenterPosition[i].x, y=g_CenterPosition[i].y;
        Sx+=x; Sy+=y; Sxx+=x*x; Sxy+=x*y; ++n;
    }
    double den = n*Sxx - Sx*Sx;
    if (std::fabs(den) < 1e-6) return 0.f;
    return (float)((n*Sxy - Sx*Sy)/den);
}


/* ---------- 底部“竖直贴边”检测 ---------- */
static int detect_stick_bottom_strict(int& r_mid) {
    r_mid = -1;
    if (g_CenterNum <= 1) return 0;

    const int LCOL=0, RCOL=MAX_VIDEO_POINT-1;
    int last = MIN(g_CenterNum-1, g_cfg.bottom_rows-1);
    if (last < 1) return 0;

    auto is_stick_row = [&](bool left, int i)->bool{
        int y = left ? g_LeftEdge[i].y : g_RightEdge[i].y;
        bool at_edge = left ? (y <= LCOL + g_cfg.stick_edge_tol)
                            : (y >= RCOL - g_cfg.stick_edge_tol);
        if (!at_edge) return false;
        if (i>0){
            int yp = left ? g_LeftEdge[i-1].y : g_RightEdge[i-1].y;
            if (ABS(y-yp) > g_cfg.stick_dy_tol) return false;
        }
        return true;
    };

    auto scan = [&](bool left, int& mid)->bool{
        int run=0,best=0,gap=0,first=0,bf=0,bl=0;
        for (int i=0;i<=last;++i){
            if (is_stick_row(left,i)){ if(run==0) first=i; run++; gap=0; }
            else if (run>0 && gap<g_cfg.stick_gap_allow){ gap++; }
            else { if (run>best){best=run; bf=first; bl=i-1;} run=0; gap=0; }
        }
        if (run>best){ best=run; bl=last; bf=first; }
        if (best >= g_cfg.stick_min_run){ mid=(bf+bl)/2; return true; }
        return false;
    };

    int midL=-1, midR=-1;
    bool L = scan(true,  midL);
    bool R = scan(false, midR);

    if (L && !R) { r_mid=midL; return -1; }
    if (R && !L) { r_mid=midR; return  1; }

    // 维持锁定，避免左右抖
    if (s_stick_latch>0) return s_stick_side;
    return 0;
}

/* ---------- "中部贴边"检测，跳过上下部分 ---------- */
static int detect_stick_middle_strict(int& r_mid) {
    r_mid = -1;
    if (g_CenterNum <= 1) return 0;

    const int LCOL=0, RCOL=MAX_VIDEO_POINT-1;
    
    // 定义上下跳过的行数
    const int SKIP_TOP_ROWS = 40;    // 跳过顶部的行数
    const int SKIP_BOTTOM_ROWS = 20; // 跳过底部的行数
    
    // 计算搜索范围
    int search_start = SKIP_BOTTOM_ROWS;
    int search_end = g_CenterNum - 1 - SKIP_TOP_ROWS;
    
    // 确保搜索范围有效
    if (search_end <= search_start) return 0;
    
    auto is_stick_row = [&](bool left, int i)->bool{
        int y = left ? g_LeftEdge[i].y : g_RightEdge[i].y;
        bool at_edge = left ? (y <= LCOL + g_cfg.stick_edge_tol)
                            : (y >= RCOL - g_cfg.stick_edge_tol);
        if (!at_edge) return false;
        if (i>0){
            int yp = left ? g_LeftEdge[i-1].y : g_RightEdge[i-1].y;
            if (ABS(y-yp) > g_cfg.stick_dy_tol) return false;
        }
        return true;
    };

    auto scan = [&](bool left, int& mid)->bool{
        int run=0,best=0,gap=0,first=0,bf=0,bl=0;
        for (int i=search_start; i<=search_end; ++i){
            if (is_stick_row(left,i)){ if(run==0) first=i; run++; gap=0; }
            else if (run>0 && gap<g_cfg.stick_gap_allow){ gap++; }
            else { if (run>best){best=run; bf=first; bl=i-1;} run=0; gap=0; }
        }
        if (run>best){ best=run; bl=search_end; bf=first; }
        if (best >= g_cfg.stick_min_run){ mid=(bf+bl)/2; return true; }
        return false;
    };

    int midL=-1, midR=-1;
    bool L = scan(true,  midL);
    bool R = scan(false, midR);

    if (L && !R) { r_mid=midL; return -1; }
    if (R && !L) { r_mid=midR; return  1; }

    // 维持锁定，避免左右抖
    if (s_stick_latch>0) return s_stick_side;
    return 0;
}

// 法一：
/* ---------- 贴边段贝塞尔补线 ---------- */
// 近弯贴边补线）：
// 端点 P0=(row=0, y=bottom_center)，另一端是“贴边段”的行向中点 P2=(row=r_mid, y=边界列)。
// 以“完整可见的一侧轨迹”（非贴边那一侧）作为参考，画一条“平行的曲线/直线”。
// 具体：默认用二次贝塞尔（更平顺）；若该侧在 [0..r_mid] 上几乎“直线”，则退化为直线。
// // 近弯补线（端点：P0=底部中点；P2=“连续贴边段”的行向中点，列即边界列）
// side: -1 左贴；+1 右贴；0 无
// mid_row: 连续贴边“运行段”的行向中点（detect_stick_* 返回的）
static void apply_near_bend_with_mid_endpoint(int side, int mid_row) {
    if (side == 0 || mid_row < 0 || g_CenterNum < 2) return;

    // 1. 确定三个控制点
    const int P0_x = 0;  // 底端行
    const int P0_y = (g_LeftEdge[0].y + g_RightEdge[0].y) / 2;  // 底部中点
    
    // 中间控制点取在 40% 处
    int P1_x = (int)(mid_row * 0.4f);
    P1_x = CLAMP(P1_x, P0_x, mid_row);
    // 中间点的y坐标:考虑对侧边缘
    int y_opp = (side < 0) ? g_RightEdge[P1_x].y : g_LeftEdge[P1_x].y;
    int P1_y = (y_opp + ((side < 0) ? 0 : MAX_VIDEO_POINT-1)) / 2;
    
    // 终点即贴边中点
    const int P2_x = mid_row;
    const int P2_y = (side < 0) ? 0 : (MAX_VIDEO_POINT - 1);

    // 2. 生成贝塞尔曲线点
    for (int i = P0_x; i <= P2_x; ++i) {
        float t = (float)(i - P0_x) / (float)(P2_x - P0_x);
        
        // 二次贝塞尔公式: B(t) = (1-t)^2*P0 + 2(1-t)t*P1 + t^2*P2
        float y = (1.f - t) * (1.f - t) * P0_y + 
                 2.f * (1.f - t) * t * P1_y +
                 t * t * P2_y;
                 
        g_CenterPosition[i].y = CLAMP((int)(y + 0.5f), 0, MAX_VIDEO_POINT-1);
        s_locked_rows[i] = true;
        s_draw_row[i] = true;
    }

    // 3. 撞边后停止绘制
    for (int i = P0_x; i <= P2_x; ++i) {
        if (g_CenterPosition[i].y <= 0 || 
            g_CenterPosition[i].y >= MAX_VIDEO_POINT-1) {
            for (int k = i+1; k < g_CenterNum; ++k) {
                s_draw_row[k] = false;
            }
            break;
        }
    }
}

// 法二：
/* 改进的近弯补线：基于非贴边一侧轨迹拟合 */
static void improved_near_bend_fitting(int side, int r_low) {
    if (side == 0 || r_low < 0 || g_CenterNum < 5) return;
    
    // 确定分析范围：从底部到贴边起始点附近
    int r_range = MIN(r_low + 10, g_CenterNum-1); // 多取几行进行拟合
    
    // 1. 获取非贴边一侧的轨迹点
    std::vector<_POINT> ref_points;
    for (int i = 0; i <= r_range; ++i) {
        _POINT pt;
        pt.x = i; // 行号
        pt.y = (side < 0) ? g_RightEdge[i].y : g_LeftEdge[i].y; // 非贴边一侧的列
        ref_points.push_back(pt);
    }
    
    // 2. 拟合非贴边一侧轨迹
    // 使用多项式拟合或贝塞尔拟合，这里用二次多项式示例
    double a = 0, b = 0, c = 0;
    if (ref_points.size() >= 3) {
        // 从参考点中均匀选取三个点进行二次拟合
        int mid_idx = ref_points.size() / 2;
        double x1 = ref_points[0].x, y1 = ref_points[0].y;
        double x2 = ref_points[mid_idx].x, y2 = ref_points[mid_idx].y;
        double x3 = ref_points[ref_points.size()-1].x, y3 = ref_points[ref_points.size()-1].y;
        
        findQuadraticCoefficients(y1, x1, y2, x2, y3, x3, a, b, c);
    } else {
        // 点数不足，退化为线性拟合
        float k = Slope_Calculate(0, ref_points.size(), ref_points.data());
        b = k;
        c = ref_points[0].y - k * ref_points[0].x;
    }
    
    // 3. 根据拟合结果生成对称的补线
    for (int i = 0; i <= r_low; ++i) {
        // 计算拟合轨迹上的点
        double fitted_y = a * i * i + b * i + c;
        
        // 计算补线点（与拟合轨迹对称的点）
        int opp_y = (side < 0) ? 0 : (MAX_VIDEO_POINT - 1); // 贴边侧边界
        int center_y = (fitted_y + opp_y) / 2; // 对称中点
        
        // 设置中心点
        g_CenterPosition[i].y = CLAMP((int)center_y, 0, MAX_VIDEO_POINT-1);
        s_locked_rows[i] = true;
        s_draw_row[i] = true;
    }
    
    // // 4. 检查是否撞边
    // stop_after_border_hit();
}


/* ---------- 十字路口：宽度突增 → 延长上一段中心线（尊重锁定，带冷却/融合） ---------- */
struct StickRun { bool found; int first; int last; int mid; };

// 跳过检测部分
// 法一：从底部搜索
static StickRun bottom_stick_run(bool left_side,
                                 int bottom_rows,   
                                 int edge_tol,      
                                 int dy_tol,        
                                 int min_run,       
                                 int gap_allow)     
{
    StickRun R{false,0,0,-1};
    if (g_CenterNum <= 1) return R;

    const int LCOL = 0, RCOL = MAX_VIDEO_POINT-1;
    int last = MIN(g_CenterNum-1, bottom_rows-1);
    auto is_stick_row = [&](int i)->bool{
        int y = left_side ? g_LeftEdge[i].y : g_RightEdge[i].y;
        bool at_edge = left_side ? (y <= LCOL + edge_tol) : (y >= RCOL - edge_tol);
        if (!at_edge) return false;
        if (i>0){
            int yp = left_side ? g_LeftEdge[i-1].y : g_RightEdge[i-1].y;
            if (ABS(y-yp) > dy_tol) return false; // 竖直一致性
        }
        return true;
    };

    int run=0, best=0, gap=0, first=0, bf=0, bl=0;
    for (int i=0;i<=last;++i){
        if (is_stick_row(i)){ if(run==0) first=i; run++; gap=0; }
        else if (run>0 && gap<gap_allow){ gap++; }
        else { if (run>best){best=run; bf=first; bl=i-1;} run=0; gap=0; }
    }
    if (run>best){ best=run; bf=first; bl=last; }

    if (best >= min_run){ R.found=true; R.first=bf; R.last=bl; R.mid=(bf+bl)/2; }
    return R;
}

// 法二：从中间搜索
static StickRun middle_stick_run(bool left_side,
                               int total_rows,    // 总行数
                               int edge_tol,      
                               int dy_tol,        
                               int min_run,       
                               int gap_allow)     
{
    StickRun R{false,0,0,-1};
    if (g_CenterNum <= 1) return R;

    
    // 计算搜索范围
    int search_start = SKIP_BOTTOM_ROWS;
    int search_end = MIN(g_CenterNum-1, total_rows-1) - SKIP_TOP_ROWS;
    
    // 确保搜索范围有效
    if (search_end <= search_start) return R;

    const int LCOL = 0, RCOL = MAX_VIDEO_POINT-1;
    
    auto is_stick_row = [&](int i)->bool{
        int y = left_side ? g_LeftEdge[i].y : g_RightEdge[i].y;
        bool at_edge = left_side ? (y <= LCOL + edge_tol) : (y >= RCOL - edge_tol);
        if (!at_edge) return false;
        if (i>0){
            int yp = left_side ? g_LeftEdge[i-1].y : g_RightEdge[i-1].y;
            if (ABS(y-yp) > dy_tol) return false; // 竖直一致性
        }
        return true;
    };

    int run=0, best=0, gap=0, first=0, bf=0, bl=0;
    for (int i=search_start; i<=search_end; ++i){
        if (is_stick_row(i)){ if(run==0) first=i; run++; gap=0; }
        else if (run>0 && gap<gap_allow){ gap++; }
        else { if (run>best){best=run; bf=first; bl=i-1;} run=0; gap=0; }
    }
    if (run>best){ best=run; bf=first; bl=search_end; }

    if (best >= min_run){ R.found=true; R.first=bf; R.last=bl; R.mid=(bf+bl)/2; }
    return R;
}


static void crossroad_handle() {
    static int cooldown = 0;
    if (g_CenterNum < 6) return;
    if (cooldown > 0) --cooldown;

    // 1. 检测左右贴边段
    StickRun L = middle_stick_run(/*left=*/true,
                   XR_BOTTOM_ROWS, XR_EDGE_TOL, XR_DY_TOL, XR_MIN_RUN, XR_GAP_ALLOW);
    StickRun R = middle_stick_run(/*left=*/false,
                   XR_BOTTOM_ROWS, XR_EDGE_TOL, XR_DY_TOL, XR_MIN_RUN, XR_GAP_ALLOW);

    // 2. 增加新的检测方法：查找宽度突然增大的位置
    int width_jump_row = -1;
    for (int i=2; i < g_CenterNum-2; ++i) {
        int w0 = g_RightEdge[i-1].y - g_LeftEdge[i-1].y;
        int w1 = g_RightEdge[i].y - g_LeftEdge[i].y;
        int w2 = g_RightEdge[i+1].y - g_LeftEdge[i+1].y;
        
        // 宽度突然增大且持续增大
        if (w1 > w0*1.2 && w2 >= w1) {
            width_jump_row = i;
            break;
        }
    }

    // 3. 判断是否是十字路口（左右同时贴边或宽度突增）
    bool is_cross = false;
    int hit = -1;
    
    // 方法一：左右同时贴边
    if (L.found && R.found && ABS(L.mid - R.mid) <= XR_ROW_TOL) {
        is_cross = true;
        hit = MAX(1, MIN(L.mid, R.mid) - 1);
    }
    // 方法二：宽度突然增大
    else if (width_jump_row > 0) {
        is_cross = true;
        hit = width_jump_row - 1;
    }

    // 4. 如果检测到十字，执行补线
    if (is_cross && cooldown == 0) {
        int dx = g_CenterPosition[hit].x - g_CenterPosition[hit-1].x; if (dx==0) dx=1;
        float k_prev = (float)(g_CenterPosition[hit].y - g_CenterPosition[hit-1].y) / (float)dx;
        int bx = g_CenterPosition[hit].x;
        int by = g_CenterPosition[hit].y;

        int to = MIN(g_CenterNum, hit + XR_EXTEND_N);
        for (int t = hit + 1; t < to; ++t) {
            if (s_locked_rows[t] || !s_draw_row[t]) continue;   
            int ddx = g_CenterPosition[t].x - bx;
            int y_ext = ROUNDI(by + k_prev * ddx);
            int y_ori = g_CenterPosition[t].y;
            int y     = ROUNDI(XR_BLEND * y_ext + (1.f - XR_BLEND) * y_ori);
            g_CenterPosition[t].y = CLAMP(y, 0, MAX_VIDEO_POINT-1);
        }
        cooldown = XR_COOL_FRM;
    }
}

/* ---------- 防突变：限幅（不外推，不拉直，尊重锁定） ---------- */
static void slope_limiter() {
    if (g_CenterNum < 3) return;
    const int W = MAX(3, g_cfg.k_win);

    float k_prev = lr_slope(0, MIN(W-1, g_CenterNum-1));
    for (int i=1; i<g_CenterNum; ++i) {
                if (s_locked_rows[i] || !s_draw_row[i]) continue;   // 锁定/不绘制的行，后续模块不再改
        // if (s_locked_rows[i]) continue;                   // 锁定行不动

        int i0 = MAX(0, i - W + 1), i1 = i;
        float k_now = lr_slope(i0, i1);

        float dk = k_now - k_prev;
        if (dk >  g_cfg.k_delta_max) dk =  g_cfg.k_delta_max;
        if (dk < -g_cfg.k_delta_max) dk = -g_cfg.k_delta_max;
        k_now = k_prev + dk;

        int dx = g_CenterPosition[i].x - g_CenterPosition[i-1].x; if (dx==0) dx=1;
        int y_est = ROUNDI(g_CenterPosition[i-1].y + k_now * dx);
        int y     = ROUNDI(0.6f * y_est + 0.4f * g_CenterPosition[i].y); // 轻融合
        g_CenterPosition[i].y = CLAMP(y, 0, MAX_VIDEO_POINT-1);

        k_prev = k_now;
    }
}

// 存储斑马线状态
struct ZebraDetector {
    bool is_zebra_crossing = false;    // 是否检测到斑马线
    int suspect_count = 0;             // 可疑行计数
    int detected_row = -1;             // 检测到斑马线的行号
    bool printed = false;              // 是否已经打印过检测信息
};

static ZebraDetector g_zebra;

// 斑马线判断函数
bool isZebraCrossing(int line) {
    // 检查是否在检测范围内
    if (line < ZEBRA_START_ROW || line >= ZEBRA_BOTTOM_ROWS) {
        return false;
    }

    // 至少需要8个白块
    if (g_SEnum < 7) return false;

    // 统计连续相似宽度的白块
    std::vector<int> widths;
    for (int i = 0; i < g_SEnum; i++) {
        widths.push_back(g_End[i] - g_Start[i]);
    }

    // 计算相邻宽度差
    int similar_count = 0;
    int max_similar_count = 0;
    const int WIDTH_DIFF_TOL = 2; // 允许的宽度差异

    for (int i = 0; i < widths.size() - 1; i++) {
        if (abs(widths[i] - widths[i + 1]) <= WIDTH_DIFF_TOL) {
            similar_count++;
            if (similar_count > max_similar_count) {
                max_similar_count = similar_count;
            }
        } else {
            similar_count = 0;
        }
    }

    // 要求至少有5个连续相似宽度的白块
    // 并且这些白块的宽度在合理范围内（8-15像素）
    if (max_similar_count >= 3) { // 连续5个相似的需要4个相似对
        // 检查这些相似宽度的白块是否在合理范围内
        int count_in_range = 0;
        for (int width : widths) {
            if (width >= 8 && width <= 15) {
                count_in_range++;
            }
        }
        
        // 至少要有6个合理宽度的白块
        return count_in_range >= 4;
    }

    return false;
}

void printZebraCrossing() {
    if (g_zebra.is_zebra_crossing && !g_zebra.printed) {
        std::cout << "\n==========================" << std::endl;
        std::cout << "检测到斑马线!" << std::endl;
        std::cout << "检测位置: 第 " << g_zebra.detected_row << " 行" << std::endl;
        std::cout << "==========================" << std::endl;
        g_zebra.printed = true;
    }
    else {
        std::cout << "未检测到斑马线" << std::endl;
        std::cout << "当前行白块数量: " << g_SEnum << std::endl;
        if (g_SEnum > 0) {
            std::cout << "白块宽度: ";
            for (int i = 0; i < g_SEnum; i++) {
                std::cout << (g_End[i] - g_Start[i]) << " ";
            }
            std::cout << std::endl;
        }
    }
}

/* ---------- 外部调用入口：在生成初始中心线后调用 ---------- */
void PostCenterProcess() {
    if (g_CenterNum < 3) return;

    // 初始化标记数组
    clear_draw_mask();  // 设置所有行默认都可绘制
    clear_locked();     // 设置所有行默认都不锁定

    // 1) 贴边检测和补线
    int r_mid = -1;
    int side = detect_stick_bottom_strict(r_mid);
    if (side != 0) { s_stick_side = side; s_stick_latch = g_cfg.stick_latch_len; }
    else if (s_stick_latch > 0) { side = s_stick_side; --s_stick_latch; }

    if (side != 0) {
        // 贴边补线
        apply_near_bend_with_mid_endpoint(side, r_mid);
        // 立即检查是否撞边
        stop_after_border_hit();
    }

    // 2) 十字路口处理
    // 确保十字路口处理函数尊重锁定和不绘制标记
    crossroad_handle();
    // 十字处理后再次检查是否撞边
    stop_after_border_hit();

// ===== 停车库期间：竖直中线锁定（进入→离开都生效） =====
{
    // 进入/离开事件闩锁：进入时锁定 anchor_x，离开时解除
    static bool s_garage_latch = false;
    static int  s_anchor_x     = -1;

    if (g_garage_event == 1) {          // 进入
        s_garage_latch = true;
        s_anchor_x     = g_garage_anchor_x;
        std::cout<<"[停车库]进入"<<std::endl;
        std::cout<<"[停车库]g_garage_anchor_x: "<<g_garage_anchor_x<<std::endl;
    } else if (g_garage_event == 2) {   // 离开
        s_garage_latch = false;
        s_anchor_x     = -1;
        std::cout<<"[停车库]离开"<<std::endl;
    }

    // // 仍在停车库期间也保持锁定（防丢事件）
    // if (g_garage_state == 1 && !s_garage_latch) {
    //     s_garage_latch = true;
    //     s_anchor_x     = g_garage_anchor_x;
    //     std::cout<<"[停车库]still in garage"<<g_garage_anchor_x<<std::endl;
    // }

    // // 需要竖直：闩锁生效时
    // if (s_garage_latch && g_CenterNum > 0) {
    //     // 锚点兜底与夹紧
    //     if (s_anchor_x < 0) s_anchor_x = (g_garage_anchor_x >= 0)
    //                                       ? g_garage_anchor_x
    //                                       : g_CenterPosition[0].x;
    //     if (s_anchor_x < 0) s_anchor_x = 0;
    //     if (s_anchor_x >= IMAGE_MIDDLE) s_anchor_x = IMAGE_MIDDLE - 1;

    //     // 关键：生成竖直线 - 固定x坐标，y从底到顶递增
    //     int rows = g_CenterNum;
    //     for (int i = 0; i < rows; ++i) {
    //         g_CenterPosition[i].x = i;  // 行号保持递增
    //         g_CenterPosition[i].y = s_anchor_x;  // 列号固定为锚点值
    //         s_draw_row[i] = true;  // 标记为可绘制
    //         s_locked_rows[i] = true;  // 锁定该行，防止被其他处理修改
    //     }

    //     // 对竖直线不进行斜率限制和平滑处理
    //     return; // 可以直接返回，跳过后续处理
    // }
}
// ===== 停车库竖直锁定结束 =====


    // 3) 防突变处理
    // 确保slope_limiter函数尊重锁定和不绘制标记
    slope_limiter();
    // 防突变处理后再次检查是否撞边
    stop_after_border_hit();

    // 4) EMA平滑
    if (g_cfg.ema_alpha > 0.f) {
        float a = CLAMP(g_cfg.ema_alpha, 0.f, 1.f);
        for (int i = 1; i < g_CenterNum; ++i) {
            // 如果该行被标记为不绘制或已锁定，则跳过
            if (!s_draw_row[i] || s_locked_rows[i]) continue;
            
            g_CenterPosition[i].y = ROUNDI(a * g_CenterPosition[i].y
                                     + (1.f - a) * g_CenterPosition[i-1].y);
            
            // 检查当前行是否撞边
            if (g_CenterPosition[i].y <= 0 || g_CenterPosition[i].y >= MAX_VIDEO_POINT-1) {
                // 标记当前行及以上行不再绘制
                for (int k = i; k < g_CenterNum; ++k) {
                    s_draw_row[k] = false;
                    s_locked_rows[k] = true;
                }
                break; // 找到撞边点后立即退出
            }
        }
    }
}

/*********      search:处理数据即测试算法的函数 ********/
void Search()
{
        // 重置斑马线检测状态
        g_zebra.is_zebra_crossing = false;
        g_zebra.suspect_count = 0;
        g_zebra.detected_row = -1;
        g_zebra.printed = false;

        g_LeftEdgeNum = 0; g_RightEdgeNum = 0;
        g_SearchFlag = 1;
        possible_cross = 0;
        int i, line, white_width = 0;

        #if USE_OPENCV
    // 使用OpenCV进行二值化（覆盖原数组，后续逻辑不变）
    cv::Mat mat(MAX_VIDEO_LINE, MAX_VIDEO_POINT, CV_8UC1, g_VideoImageData);
    cv::Mat bin;
    if (IMAGE_MIDDLE < 0) IMAGE_MIDDLE = 128;
    cv::threshold(mat, bin, IMAGE_MIDDLE, 255, cv::THRESH_BINARY);
    // 轻度开运算去噪
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
    std::memcpy(g_VideoImageData, bin.data, MAX_VIDEO_LINE*MAX_VIDEO_POINT*sizeof(uint8));
        #endif


        //=======================图像识别=========================//
        //PS：此时我们的图像已经提取完毕 我们从底部开始，向上识别图像
        //图像识别数据初始化

        g_CenterNum = 0;
        g_LeftEdgeNum = 0;
        g_RightEdgeNum = 0;
        g_SearchFlag = 1;

        for (line = 0; line <= MAX_VIDEO_LINE - 1; line++)
        {
                //首先 求出本行所有白块的位置
                g_SEnum = 0;  //本行的白块计数器清零
                if (g_VideoImageData[line][MAX_VIDEO_POINT - 1] > IMAGE_MIDDLE)  //大于阈值为白 小于阈值为黑
                        //如果最左侧是白块，则计入（因为下面的代码处理不到最左侧）
                        g_Start[g_SEnum] = 0;
                //从左向右处理
                for (i = MAX_VIDEO_POINT - 1; i > 0; i--)
                {
                        //右白左黑 则为左边界
                        if (g_VideoImageData[line][i] > IMAGE_MIDDLE && g_VideoImageData[line][i - 1] <= IMAGE_MIDDLE)
                        {
                                //g_End[g_SEnum]和g_VideoImageData的第二维是反着的
                                g_End[g_SEnum] = MAX_VIDEO_POINT - i;
                                //计数 
                                g_SEnum++;
                        }
                        //左白右黑 则为右边界
                        else if (g_VideoImageData[line][i] <= IMAGE_MIDDLE && g_VideoImageData[line][i - 1] > IMAGE_MIDDLE)
                        {
                                g_Start[g_SEnum] = MAX_VIDEO_POINT - i - 1;
                                //这里就没必要计数了，因为有左边界则一定会出现右边界，而且是交替出现的
                        }
                        //边界数太多就没必要浪费时间，直接跳过
                        else if (g_SEnum >= 10)
                        {
                                break;
                        }
                }
                //最右侧白块判断
                if (g_VideoImageData[line][0] > IMAGE_MIDDLE && g_SEnum < 10)
                {
                        //如果最右边也是白的，则增加一个新的右边界
                        g_End[g_SEnum] = MAX_VIDEO_POINT - 1;
                        g_SEnum++;
                }
                //把最下面的那一行单独拿出来处理 做为后续处理的基础
                if (g_SearchFlag)                    //最下一行处理标志
                {
                        int temp_mark = 0;
                        if (g_SEnum == 0)
                        {
                                continue;
                        }
                        //白块长度计算
                        white_width = g_End[0] - g_Start[0];
                        //获取两点间最宽的白块，可以认为是道路
                        for (i = 1; i < g_SEnum; i++)  //直接求最宽的白块
                        {
                                if (g_End[i] - g_Start[i] >= white_width)
                                {
                                        white_width = g_End[i] - g_Start[i];
                                        temp_mark = i;
                                }
                        }
                        if (white_width > WIDTH)
                        {
                                g_SearchFlag = 0;
                                //认为第一行最宽的白块便是所需白块
                //向上为x，向右为y
                //存入当前行的左边界和右边界y坐标
                                g_LeftEdge[g_LeftEdgeNum].x = line;
                                g_LeftEdge[g_LeftEdgeNum++].y = g_Start[temp_mark];
                                g_RightEdge[g_RightEdgeNum].x = line;
                                g_RightEdge[g_RightEdgeNum++].y = g_End[temp_mark];
                        }
                }
                else
                {
                        if (g_SEnum == 0)
                        {
                                break;
                        }
                        for (i = 0, g_CoverNum = 0; i < g_SEnum; i++)
                        {
                                //覆盖关系，因为图像是一个联通区域 于是找与上一行已经找到白块相联通的白块
                //g_Cover = 当前行和上一行右边界的最小值 - 当前行和上一行左边界的最大值
                //通常来说越往上白块长度越短
                                g_Cover = MIN(g_End[i], g_RightEdge[g_RightEdgeNum - 1].y) - MAX(g_Start[i], g_LeftEdge[g_LeftEdgeNum - 1].y);
                                if (g_Cover > -1)
                                {
                                        g_CoverIndex[g_CoverNum] = i;
                                        g_CoverNum++;
                                }
                        }
                        if (g_CoverNum == 1) //如果只有一个联通的 直接取出这个联通白块
                        {
                                g_LeftEdge[g_LeftEdgeNum].x = line;
                                g_LeftEdge[g_LeftEdgeNum++].y = g_Start[g_CoverIndex[0]];
                                g_RightEdge[g_RightEdgeNum].x = line;
                                g_RightEdge[g_RightEdgeNum++].y = g_End[g_CoverIndex[0]];
                                const int EDGE_MARGIN = 2; // 边界容差
                                bool left_close = (g_LeftEdge[g_LeftEdgeNum-1].y <= EDGE_MARGIN);
                                bool right_close = (g_RightEdge[g_RightEdgeNum-1].y >= MAX_VIDEO_POINT-1-EDGE_MARGIN);
                                //可以结合边界跳变判断
                                //possible_cross = 1;
                                if(left_close && right_close && line > 10){
                                        g_connect_edge_count++; 
                                        if(g_connect_edge_count == 1){
                                                g_connect_edge_start = line - 1; 
                                        }
                                        if(((g_connect_edge_count > 2 && line > 80) || (g_connect_edge_count > 4 && line > 60))&& g_connect_edge_start > 15){
                                                possible_cross = true;
                                        }
                                }else{
                                        g_connect_edge_count = 0;
                                }
                        }
                        else if (g_CoverNum == 0)  //没有联通的白块 说明图像已经处理完毕
                        {
                                break;                 //跳出整个图像处理的循环 进入求中心部分
                        }
                        // 在Search()函数中修改多白块选择逻辑

else //有多个白块   需要取舍
{
    // 原始逻辑：简单选择与上一行白块中心最接近的白块
    // int temp_mark, temp_dis, temp_center, last_center;
    // temp_mark = g_CoverIndex[0];
    // temp_center = (g_Start[g_CoverIndex[0]] + g_End[g_CoverIndex[0]]) / 2;
    // last_center = (g_LeftEdge[g_LeftEdgeNum - 1].y + g_RightEdge[g_RightEdgeNum - 1].y) / 2;
    // temp_dis = ABS(last_center - temp_center);
    // for (int i = 1; i < g_CoverNum; i++)
    // {
    //     temp_center = (g_Start[g_CoverIndex[i]] + g_End[g_CoverIndex[i]]) / 2;
    //     if (ABS(temp_center - last_center) < temp_dis)
    //     {
    //         temp_dis = ABS(temp_center - last_center);
    //         temp_mark = g_CoverIndex[i];
    //     }
    // }

    // 新逻辑：检测是否可能是十字路口，如是则选择靠近边界延长线中心的白块
    int temp_mark;
    
    // 先判断是否可能处于十字路口

    
    // 1. 检查是否左右两侧均接近边界（十字特征之一）

                // 如果不是十字，仍然使用原来的策略：选择与上一行中心最接近的
        int temp_dis, temp_center, last_center;
        temp_mark = g_CoverIndex[0];
        temp_center = (g_Start[g_CoverIndex[0]] + g_End[g_CoverIndex[0]]) / 2;
    if (possible_cross) {
        // 计算左右边界的延长线位置
        float left_k = 0, right_k = 0;
        
        // 使用最近几行计算边界的斜率
        const int SLOPE_ROWS = 10;
        if (g_LeftEdgeNum >= SLOPE_ROWS) {
                        int start_idx = 0;
                        for(int i = 0; i < g_LeftEdgeNum; i++){
                                if(g_LeftEdge[i].y > 2){
                                        start_idx = i;
                                        break;
                                }
                        }

            left_k = Slope_Calculate(start_idx, start_idx + SLOPE_ROWS, g_LeftEdge);
        }
        if (g_RightEdgeNum >= SLOPE_ROWS) {
                        int start_idx = 0;
            for(int i = 0; i < g_RightEdgeNum; i++){
                                if(g_RightEdge[i].y < 159){
                                        start_idx = i;
                                        break;
                                }
                        }
            right_k = Slope_Calculate(start_idx, start_idx + SLOPE_ROWS, g_RightEdge);
        }
        
        // 根据斜率预测当前行的左右边界位置
        int predicted_left = g_LeftEdge[0].y + 
                             left_k * (line - g_LeftEdge[0].x);
        int predicted_right = g_RightEdge[0].y + 
                              right_k * (line - g_RightEdge[0].x);
        
        // 预测的中心线位置
        last_center = (predicted_left + predicted_right) / 2;
    }
    else {

        last_center = (g_LeftEdge[g_LeftEdgeNum - 1].y + g_RightEdge[g_RightEdgeNum - 1].y) / 2;
    }
        temp_dis = ABS(last_center - temp_center);
        
        for (int i = 1; i < g_CoverNum; i++) {
                temp_center = (g_Start[g_CoverIndex[i]] + g_End[g_CoverIndex[i]]) / 2;
                if (ABS(temp_center - last_center) < temp_dis) {
                        temp_dis = ABS(temp_center - last_center);
                        temp_mark = g_CoverIndex[i];
                }
        }
    
    // 保存找到的边界
    g_LeftEdge[g_LeftEdgeNum].x = line;
    g_LeftEdge[g_LeftEdgeNum++].y = g_Start[temp_mark];
    g_RightEdge[g_RightEdgeNum].x = line;
    g_RightEdge[g_RightEdgeNum++].y = g_End[temp_mark];

    // 在处理完白块后添加斑马线检测
        if (line >= ZEBRA_START_ROW && line < ZEBRA_BOTTOM_ROWS) {
            if (isZebraCrossing(line)) {
                g_zebra.suspect_count++;
                if (g_zebra.suspect_count >= ZEBRA_CONFIRM_ROWS) {
                    g_zebra.is_zebra_crossing = true;
                    if (g_zebra.detected_row == -1) {
                        g_zebra.detected_row = line;
                    }
                }
            } else {
                g_zebra.suspect_count = 0;
            }
        }
    
    // 在Search()函数结束前打印检测结果
    // printZebraCrossing();
        }
        }
}

        //=======================起点判断========================//
        //=======================十字处理========================//
        //=======================中心求取========================//
        /*
        *
        此处的中心求取非常简单粗糙，因为之前在边沿搜索的时候并没有区分两边，
        而是直接把一个白块的start认为是左边，end认为是右边 故中心数将与左右
        边的数相等，而中心位置直接由左右边相加除2所得
        *
        */

        // —— 生成中心（改：用左右边缘数量的 min）——
        // 2) 初始中心线（左右中点），务必用两侧数量的最小值
        g_CenterNum = MIN(g_LeftEdgeNum, g_RightEdgeNum);
        for (int i = 0; i < g_CenterNum; ++i) {
                g_CenterPosition[i].x = g_RightEdge[i].x;                          // 行号
                g_CenterPosition[i].y = (g_LeftEdge[i].y + g_RightEdge[i].y) / 2;  // 列（中点）
        }

        // 3) 增强处理：贴边贝塞尔补线 + 十字延长 + 防突变限幅
        PostCenterProcess();

        // 在Search()函数的最后或任何使用中心线的地方添加这段代码
// 移除不应绘制的点
for (int i = 0; i < g_CenterNum; ++i) {
    if (!s_draw_row[i]) {
        // 根据需要处理不绘制的点，比如设置为特殊值或从数组中移除
        // 将y值设为无效值
        g_CenterPosition[i].y = -1;  // 设-1是一个不会被处理的值
    }
}

        // 或在计算控制量时忽略这些点
        g_DirectionControlWhole = 0;
        g_DirectionControlLine = 0;
        for (i = 0; i < g_CenterNum; i++) {
                // 只考虑应该绘制的点，且点在有效范围内
                if (s_draw_row[i] && g_CenterPosition[i].y >= 0 && g_CenterPosition[i].y < MAX_VIDEO_POINT) {
                        g_DirectionControlLine += (int)g_CenterPosition[i].x;
                        g_DirectionControlWhole += (int)g_CenterPosition[i].y * g_CenterPosition[i].x;
                }
        }
        
        //===================控制中心及电机控制====================//
   /*   //加权平均法求方向控制
        g_DirectionControlWhole = 0;
        g_DirectionControlLine = 0;
        for (i = 0; i < g_CenterNum; i++)
        {
                if (g_CenterPosition[i].y >= 0 && g_CenterPosition[i].y <= MAX_VIDEO_POINT)
                {
                        g_DirectionControlLine += (int)g_CenterPosition[i].x;
                        g_DirectionControlWhole += (int)g_CenterPosition[i].y * g_CenterPosition[i].x;  //注意数据不要溢出
                }
        }
        if (g_DirectionControlLine > 0)
        {
                g_DirectionControl = g_DirectionControlWhole / g_DirectionControlLine;
        }
        //方向控制限定
        if (g_DirectionControl < 0)
        {
                g_DirectionControl = 0;
        }
        else if (g_DirectionControl > MAX_VIDEO_POINT)
        {
                g_DirectionControl = MAX_VIDEO_POINT;
        }
        if (ABS(g_DirectionControl - g_FormerDirectionControl) > 90)
        {
                g_DirectionControl = g_FormerDirectionControl;
        }
                */
             
        #define Y_PEAK 60              // 权重最大行
        #define WEIGHT_MAX 1.0f        // 权重峰值
        #define WEIGHT_MIN 0.3f        // 边缘最小权重，可调

        // ------------------------
        // 方向控制计算
        // ------------------------
        g_DirectionControlWhole = 0;
        g_DirectionControlLine = 0;

        for (int i = 0; i < g_CenterNum; i++)
        {
            int x = g_CenterPosition[i].x;
            int y = g_CenterPosition[i].y;

            if (y >= 0 && y <= MAX_VIDEO_LINE)
            {

                float dy = (float)(y - Y_PEAK);
                float range = (float)Y_PEAK;
                float w = 1.0f - (dy * dy) / (range * range); // 抛物线
                if (w < 0) w = 0;
                float weight = WEIGHT_MIN + w * (WEIGHT_MAX - WEIGHT_MIN);
                // --------------------------------------

                g_DirectionControlLine  += (int)(x * weight);
                g_DirectionControlWhole += (int)(y * x * weight);
            }
        }

        if (g_DirectionControlLine > 0)
            g_DirectionControl = g_DirectionControlWhole / g_DirectionControlLine;

        // 限制范围
        if (g_DirectionControl < 0)
            g_DirectionControl = 0;
        else if (g_DirectionControl > MAX_VIDEO_POINT)
            g_DirectionControl = MAX_VIDEO_POINT;

        // 大跳变保护
        if (ABS(g_DirectionControl - g_FormerDirectionControl) > 90)
            g_DirectionControl = g_FormerDirectionControl;
       /*                // 可调参数：试验时改这三个值
        #define FAR_SCALE        0.6f   // 远端(高 y)权重乘子，0.0 ~ 1.0，越小远端影响越低
        #define NEAR_SCALE       0.8f   // 近端(低 y)权重乘子，0.0 ~ 1.0，可防止近端抖动影响过大
        #define MIN_WEIGHT       0.05f  // 权重下限，避免除零和完全忽略某些行

        // 计算中点（基于行数 MAX_VIDEO_LINE）
        int mid_line = 44;

        // 使用浮点累加，最后再赋回全局（如果你全局用整数可 cast）
        float acc_num = 0.0f;   // 分子累加 x * w
        float acc_den = 0.0f;   // 权重之和

        for (i = 0; i < g_CenterNum; i++)
        {
            int y = g_CenterPosition[i].y;
            int x = g_CenterPosition[i].x;

            // 只统计有效行（用 MAX_VIDEO_LINE 而不是 MAX_VIDEO_POINT）
            if (y >= 0 && y < MAX_VIDEO_LINE)
            {
                // 标准化到 [-1, 1]（越接近0越靠中间）
                float norm = 0.0f;
                if (mid_line > 0)
                    norm = (float)(y - mid_line) / (float)mid_line;  // -1 .. 1

                // 二次倒 U 型权重（中间最大）
                float w = 1.0f - norm * norm;   // 1 at mid, 0 at extremes

                // 对远/近端进行额外缩放（如果你想让远处影响更小就把 FAR_SCALE 调小）
                if (y > mid_line)
                {
                    // 远端（上半部分）做进一步衰减
                    w *= FAR_SCALE;
                }
                else
                {
                    // 近端（下半部分）也略微衰减，防止地面噪点主导
                    w *= NEAR_SCALE;
                }

                // 权重下限保护
                if (w < MIN_WEIGHT) w = MIN_WEIGHT;

                acc_num += (float)x * w;
                acc_den += w;
            }
        }

        // 计算结果并限制范围
        if (acc_den > 0.0f)
        {
            float newDir = acc_num / acc_den;
            // 限制到像素范围（你原来用 MAX_VIDEO_POINT 作为 x 最大）
            if (newDir < 0.0f) newDir = 0.0f;
            if (newDir > (float)MAX_VIDEO_POINT) newDir = (float)MAX_VIDEO_POINT;

            // 平滑/限幅（把 90 改小，让反应更灵敏也不剧烈）
            float max_delta = 90.0f; // 可调：每帧最大允许的改变（像素）
            if (fabsf(newDir - (float)g_FormerDirectionControl) > max_delta)
            {
                if (newDir > (float)g_FormerDirectionControl)
                    newDir = (float)g_FormerDirectionControl + max_delta;
                else
                    newDir = (float)g_FormerDirectionControl - max_delta;
            }

            g_DirectionControl = (int)(newDir + 0.5f);
        }
        else
        {
            // 没有有效权重时保持原值
            g_DirectionControl = g_FormerDirectionControl;
        }*/
}

/**
* @brief 二次函数拟合
* @param double y1 x1                第一个点坐标
* @param double y2 x2                第二个点坐标
* @param double y3 x3                第三个点坐标
* @param double &a &b &b             接收返回值，ax^2 + bx + c
* @see CTest       Slope_Calculate(start, end, border);//斜率
*/
void findQuadraticCoefficients(double y1, double x1, double y2, double x2, double y3, double x3, double &a, double &b, double &c) {
    double A[3][3] = {
        {x1 * x1, x1, 1},
        {x2 * x2, x2, 1},
        {x3 * x3, x3, 1}
    };
    double B[3] = { y1, y2, y3 };
    //使用克拉默法则求解
    double detA = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
        A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
        A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    double detA_a = B[0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
        A[0][1] * (B[1] * A[2][2] - B[2] * A[1][2]) +
        A[0][2] * (B[1] * A[2][1] - B[2] * A[1][1]);

    double detA_b = A[0][0] * (B[1] * A[2][2] - B[2] * A[1][2]) -
        B[0] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
        A[0][2] * (A[1][0] * B[2] - B[1] * A[2][0]);
    /*
    double detA_c = A[0][0] * (A[1][1] * B[2] - A[1][2] * B[1]) -
        A[0][1] * (A[1][0] * B[2] - A[1][2] * B[0]) +
        B[0] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    */
    // ax^2 + bx + c
    a = detA_a / detA;
    b = detA_b / detA;
    c = y1 - (a * x1 * x1 + b * x1);
}

/**
* @brief 最小二乘法
* @param uint8 begin                输入起点
* @param uint8 end                  输入终点
* @param uint8 *border              输入需要计算斜率的边界首地址
*  @see CTest       Slope_Calculate(start, end, border);//斜率
*/
float Slope_Calculate(uint8 begin, uint8 end, _POINT* border)
{
    float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
    int16 i = 0;
    float result = 0;
    static float resultlast;

    for (i = begin; i < end; i++)
    {
        xsum += i;
        ysum += border[i].y;
        xysum += i * (border[i].y);
        x2sum += i * i;

    }
    if ((end - begin) * x2sum - xsum * xsum) //判断除数是否为零
    {
        result = ((end - begin) * xysum - xsum * ysum) / ((end - begin) * x2sum - xsum * xsum);
        resultlast = result;
    }
    else
    {
        result = resultlast;
    }
    return result;
}