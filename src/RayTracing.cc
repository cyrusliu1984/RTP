#include <Eigen/Dense>
#include <vector>

#include "Logger.h"
#include "RayTracing.h"
#include "utils.h"


namespace VRT
{
    // 常量
    constexpr int Nx = 64;
    constexpr int Ny = 64;
    constexpr int N = Nx * Ny;
    constexpr double DISTANCE_OFFSET = 240; // 当前雷达固定的240ns距离偏移

    constexpr double deltaTheta = 0.15e-3;    // 角间距（弧度）
    constexpr double n = 1.81297;             // 折射率
    constexpr double z_laser = 0.699;        // 0922！
    constexpr double alpha = M_PI * 28 / 180; // 棱镜顶角（弧度）
    constexpr double k1 = 1.0 / n;

    constexpr double eps = 1e-20;

    // 面与起点/参考点
    const Eigen::Vector3d M0(0.0, 0.0, z_laser); // 激光出光点与棱镜中心位置的光程；709.24mm
    const Eigen::Vector3d N1(0.0, 0.0, 1.0);  // 0922！
    const Eigen::Vector3d P1(0.0, 0.0, 0.0);
    const double sinAlpha = std::sin(VRT::alpha), cosAlpha = std::cos(VRT::alpha);
    const Eigen::Vector3d P2(0.0, 0.0, - 0.0136); // 0922！

    // 第二面法向量
    const double sA = std::sin(VRT::alpha), cA = std::cos(VRT::alpha);

    // 将角度(beta, alpha)转换为三维单位向量 [x,y,z]，并对结果单位化
    Eigen::MatrixXd Unit_Angle_x(const Eigen::VectorXd &beta, const Eigen::VectorXd &alpha)
    {
        int n = beta.size();
        Eigen::MatrixXd angle(n, 3); // 每行对应一个三维方向向量

        // 元素级运算（利用Eigen的.array()实现逐元素操作）
        angle.col(0) = beta.array().sin() * alpha.array().cos(); // x分量
        angle.col(1) = beta.array().sin() * alpha.array().sin(); // y分量
        angle.col(2) = beta.array().cos();                       // z分量

        // 对每一行（每个三维向量）进行单位化
        for (int i = 0; i < n; ++i)
        {
            // 避免零向量导致的除零错误
            if (angle.row(i).norm() > 1e-10)
            {                             // 检查向量模长是否足够大
                angle.row(i).normalize(); // 原地单位化当前行向量
            }
            else
            {
                // 处理接近零向量的情况（可根据需求修改默认值）
                angle.row(i) = Eigen::Vector3d::Zero(); // 或设置为某个默认方向
            }
        }

        return angle;
    }

    // 计算探测器各像素的初始方向向量
    // 1. 
    // 依 MATLAB: rot90(A,1)（逆时针90°）的下标映射：A_rot(i,j) = A_orig(j, n - i + 1)
    // 这里使用 0-based：A_rot(i,j) = A_orig(j, n-1-i)
    static void rot90_ccw_inplace(Eigen::MatrixXd &X, Eigen::MatrixXd &Y){
        const int n = X.rows();
        const int m = X.cols();
        // 本函数假定方阵 n==m（与 MATLAB 代码使用 64×64 一致）
        assert(n == m);

        Eigen::MatrixXd Xo = X, Yo = Y; // 备份原矩阵
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < m; ++j){
                X(i,j) = Xo(j, n-1-i);
                Y(i,j) = Yo(j, n-1-i);
            }
    }

    // 2. 
    // 仿 MATLAB：reshape(v, [Nx,Ny]) → .' → flipud → fliplr → .' → (:)
    // 注意：Eigen 默认列主序，与 MATLAB 的列主序一致
    static Eigen::VectorXd matlab_channel_reorder(const Eigen::VectorXd& v, int Nx, int Ny) {
        // v 是按列展开的 Nx*Ny×1 向量（等价 MATLAB 的 (:))
        Eigen::Map<const Eigen::MatrixXd> M(v.data(), Nx, Ny); // reshape(A0_x,[Nx,Ny])
        Eigen::MatrixXd A12 = M.transpose();                   // '
        Eigen::MatrixXd A13 = A12.colwise().reverse();         // flipud
        Eigen::MatrixXd A14 = A13.rowwise().reverse();         // fliplr
        Eigen::MatrixXd A15 = A14.transpose();                 // '
        Eigen::VectorXd out(Eigen::Map<Eigen::VectorXd>(A15.data(), A15.size())); // (:)
        return out;
    }

    // 3.
    Eigen::MatrixXd calUnitVectorA0(int Nx, int Ny, double delta_theta)
    {
        // —— 1) 生成 A_i_j_90 = [ (i-Nx_M), (j-Ny_M) ]，与 MATLAB 的 1-based 等价 ——
        double Nx_M, Ny_M;
        if (((Nx - 1) % 2) == 0) {
            Nx_M = (Nx - 1) / 2.0 + 1.0;
            Ny_M = (Ny - 1) / 2.0 + 1.0;
        } else {
            Nx_M = std::floor((Nx - 1) / 2.0) + 0.5 + 1.0;
            Ny_M = std::floor((Ny - 1) / 2.0) + 0.5 + 1.0;
        }

        // 用两张表存 cell 里每个元素的 (x, y)
        Eigen::MatrixXd X(Nx, Ny), Y(Nx, Ny);
        for (int i = 0; i < Nx; ++i) {
            for (int j = 0; j < Ny; ++j) {
                // +1 将 0-based 转为 MATLAB 1-based 再减中心
                X(i, j) = (i + 1) - Nx_M;
                Y(i, j) = (j + 1) - Ny_M;
            }
        }

        // —— 2) A_i_j = rot90(A_i_j_90, 1)（逆时针90°） ——
        // 与 MATLAB 一致：对 X、Y 同时做 rot90
        rot90_ccw_inplace(X, Y);

        // —— 3) 逐点计算 beta_z_0、theta_x_0（与象限分支完全一致） ——
        Eigen::MatrixXd beta_z_0(Nx, Ny), theta_x_0(Nx, Ny);
        const double tdt = std::tan(delta_theta);

        for (int i = 0; i < Nx; ++i) {
            for (int j = 0; j < Ny; ++j) {
                const double x = X(i, j);
                const double y = Y(i, j);
                const double r = std::hypot(x, y);

                beta_z_0(i, j) = std::atan(r * tdt);  // 与 z 轴夹角

                if (r == 0.0) {
                    theta_x_0(i, j) = 0.0;
                } else {
                    const double s = y / r;
                    if (x >= 0 && y >= 0) {
                        theta_x_0(i, j) = std::asin(s);
                    } else if (x <= 0 && y >= 0) {
                        theta_x_0(i, j) = M_PI - std::asin(s);
                    } else if (x <= 0 && y <= 0) {
                        theta_x_0(i, j) = M_PI + std::abs(std::asin(s));
                    } else { // x>=0 && y<0
                        theta_x_0(i, j) = std::asin(s);
                    }
                }
            }
        }

        // —— 4) beta_z = beta_z_0'；theta_x = theta_x_0'（先转置，再按列展开） ——
        Eigen::MatrixXd beta_z = beta_z_0.transpose();
        Eigen::MatrixXd theta_x = theta_x_0.transpose();

        // 按列展平（与 MATLAB 的 (:) 一致）
        Eigen::Map<Eigen::VectorXd> beta_flat(beta_z.data(), beta_z.size());
        Eigen::Map<Eigen::VectorXd> theta_flat(theta_x.data(), theta_x.size());

        // —— 5) beta = beta_z(:) - pi；alpha = theta_x(:) ——
        Eigen::VectorXd beta = beta_flat.array() - M_PI;
        Eigen::VectorXd alpha = theta_flat;

        // —— 6) A0_x/y/z = [sin(beta).*cos(alpha), sin(beta).*sin(alpha), cos(beta)] ——
        Eigen::ArrayXd sb = beta.array().sin();
        Eigen::ArrayXd ca = alpha.array().cos();
        Eigen::ArrayXd sa = alpha.array().sin();

        Eigen::VectorXd A0_x = (sb * ca).matrix();
        Eigen::VectorXd A0_y = (sb * sa).matrix();
        Eigen::VectorXd A0_z = beta.array().cos().matrix();

        // —— 7) 对每个通道做 MATLAB 同序列：reshape→'→flipud→fliplr→'→(:) ——
        // MATLAB 代码里 reshape([64,64])；这里用 (Nx,Ny) 泛化
        Eigen::VectorXd A16 = matlab_channel_reorder(A0_x, Nx, Ny);
        Eigen::VectorXd A26 = matlab_channel_reorder(A0_y, Nx, Ny);
        Eigen::VectorXd A36 = matlab_channel_reorder(A0_z, Nx, Ny);

        // —— 8) 拼成 (Nx*Ny)×3，等价 A0=[A16,A26,A36] ——
        Eigen::MatrixXd A0(Nx * Ny, 3);
        A0.col(0) = A16;
        A0.col(1) = A26;
        A0.col(2) = A36;

        return A0;
    }

    // A0
    const Eigen::MatrixXd& getA0(){
        // static 只初始化一次 const 初始化后为只读
        static const Eigen::MatrixXd A0 = calUnitVectorA0(Nx, Ny, deltaTheta);
        return A0;
    }

    void VRT(const Eigen::MatrixXd &A0, double theta, Eigen::MatrixXd &M2, Eigen::MatrixXd &A2)
    {
        // ---------- 与第一面 (z=0) 的交点 M1 ----------
        // 平面: N1·X + D = 0, D = -N1·P1
        // t = -(N1·M0 + D) / (N1·A0) = - z_laser/A0z 
        // M1 = M0 + t * A0
        Eigen::MatrixXd M1(N, 3);
        {
            Eigen::ArrayXd denom = A0.col(2).array(); // A0z

            Eigen::ArrayXd t(N);
            t = (denom.abs() > eps).select(- z_laser / denom, 0.0); 
            Eigen::ArrayXXd trep = t.replicate(1, 3);
            M1 = (M0.transpose().replicate(N, 1).array() + trep * A0.array()).matrix();
        }

        // ---------- 第一面折射（入棱镜）A1 ----------
        // A1 = (1/n) * A0 + (cosθ₂ - (1/n)*cosθ₁) * N1; //切向连续，法向调整
        // cosθ₁ = A0·N1 = -A0z // match 0917 石博对于N1的修改 0 0 1 -> 0 0 -1 
        // cosθ₂ = sqrt(1 - sin²θ₂) = sqrt(1 - (n₁/n₂)²·sin²θ₁) = sqrt(1 - (n₁/n₂)²·(1 - cos²θ₁))
        // 当入射角过大时，(n₁/n₂)²·sin²θ₁ 可能大于 1，导致根号内为负数,
        // max(0.0) 确保在这种情况下取 0，避免数学错误,物理上，这对应全反射情况，光线无法进入棱镜
        Eigen::MatrixXd A1(N, 3);
        {
            Eigen::ArrayXd cosTheta1 = A0.col(2).array(); // 0920！！
            Eigen::ArrayXd cosTheta2 = (1.0 - k1 * k1 * (1 - cosTheta1.square())).max(0.0).sqrt(); // 0920!!
            A1.col(0) = (k1 * A0.col(0).array()).matrix();
            A1.col(1) = (k1 * A0.col(1).array()).matrix();
            A1.col(2) = (k1 * A0.col(2).array() - (cosTheta2 + k1 * cosTheta1)).matrix(); // match 0917 石博对于N1的修改 0 0 1 -> 0 0 -1 
        }

        // ---------- 与第二面 (P2, N2) 的交点 M2 ----------
        // 平面: N2·X + D = 0, D = -N2·P2
        //
        // t = -(N2·M1 + D) / (N2·A1)
        // M2 = M1 + t * A1
        const double cosTheta = std::cos(theta), sinTheta = std::sin(theta);
        // N2:
        const Eigen::Vector3d N2(-cosTheta * sinAlpha, -sinTheta * sinAlpha, cosAlpha); // match 0917 石博对于N2的修改 [cs, ss, c] -> [-cs, -ss, c]
        // Eigen::MatrixXd M2(N, 3);
        {
            const double D = -N2.dot(P2);
            Eigen::ArrayXd denom = (A1 * N2).array(); // 分母 行点乘 A1 4096,3 N2 3,1 -> 4096,1
            Eigen::ArrayXd numer = (M1 * N2).array(); // 分子 行点乘 M1 4096,3 N2 3,1 -> 4096,1
            numer.array() += D;

            Eigen::ArrayXd t(N);
            t = (denom.abs() > eps).select(-numer / denom, 0.0); // max(0.0) 确保在这种情况下取 0，避免数学错误,物理上，这对应全反射情况，光线无法进入棱镜
            Eigen::ArrayXXd trep = t.replicate(1, 3);            // 4096,1 -> 4096,3
            M2 = (M1.array() + trep * A1.array()).matrix();
        }

        // ---------- 第二面折射（出棱镜）A2 ----------
        // A2 = n*A1 + (cosθ₂ - n*cosθ₁) * N2
        // cosθ₁ = A1·N2
        // cosθ₂ = sqrt(1 - n²·sin²θ₁) = sqrt(1 - n²·(1 - cos²θ₁))
        // Eigen::MatrixXd A2(N, 3);
        {
            Eigen::ArrayXd cosTheta1 = (A1 * N2).array();                                                     // A1·N2 4096,3 x 3,1 -> 4096, 1
            Eigen::ArrayXd cosTheta2 = (1.0 - n * n * (1 - cosTheta1.square())).max(0.0).sqrt();              // 4096, 1
            Eigen::ArrayXd term = cosTheta2 + n * cosTheta1;                                                // 4096, 1
            A2 = ((n * A1).array() - term.replicate(1, 3) * N2.transpose().replicate(N, 1).array()).matrix(); // N2 3,1 transpose 1,3 replicate 4096,3
        }
    }

    PointCloudData processDecodeData(const PulseData& pulse, const int gpsWeek, const int secondInWeek, const int timeRes){
        thread_local uint16_t decData[4096]; 
        Decode::rw_probe_decode3(pulse.echoData.data(), decData); // 0922 新增解码函数
        // Decode::rotate180_rowmajor(decData);

        std::vector<Eigen::Vector3d> curPulseData;
        std::vector<double> curTimeData;
        // 1. 处理时间戳(得到毫秒时间戳)
        const int64_t pulseTimestamp = TimeUtils::gpsTimeToLinuxTimestamp(gpsWeek, secondInWeek, TimeUtils::TimeUnit::Milliseconds);

        // 2. 处理点云
        auto distancetMap = Eigen::Map<const Eigen::Array<unsigned short, Eigen::Dynamic, 1>>(decData, 4096).cast<double>(); // 4096,1
        Eigen::Array<bool, Eigen::Dynamic, 1> mask = (distancetMap < pulse.disEnd - pulse.disStart) && (distancetMap > 30);
        int validCount = mask.count();
        curPulseData.reserve(validCount); // 预分配空间，避免多次分配
        curTimeData.reserve(validCount);

        // 3. 计算出射光线
        Eigen::MatrixXd M2(N, 3), A2(N, 3); // 4096,3
        double theta = (pulse.razorCnt & 0x7FFFFFFF) / 1100800.0 * 2.0 * M_PI;
        theta = abs(2.0 * M_PI - theta); // 1020 这里的码盘与定义的码盘旋转方向相反
        VRT::VRT(getA0(), theta, M2, A2); 

        // 4. 计算距离
        Eigen::ArrayXd distance(4096);
        distance = ((distancetMap + pulse.disStart + DISTANCE_OFFSET) * timeRes * 0.001) * 0.15 ;

        // 5. 计算点云坐标
        Eigen::MatrixXd points = (M2.array() + distance.replicate(1, 3) * A2.array());
        
        // 6. 根据mask筛选有效点和时间戳
        for (int i = 0; i < N; ++i)
        {
            if (mask(i)) // TODO 0905 test 2
            {
                // 计算点
                curPulseData.emplace_back(points.row(i)); // emplace_back 将Vector3d直接构造在容器内，避免拷贝
                // 计算时间戳
                curTimeData.push_back(pulse.razorTick * 1e-3); // 微秒转毫秒
            }
        }

        return PointCloudData(pulseTimestamp, std::move(curPulseData), std::move(curTimeData));
    }
}


namespace Decode
{
    constexpr int cujishu[2048] = { 0, 1, 1496, 2, 47, 1497, 1912, 3, 678, 48, 742, 1498, 945, 1913, 243, 4, 1361, 679, 580, 49, 1389, 743,
   639, 1499, 1882, 946, 1800, 1914, 961, 244, 1543, 5, 121, 1362, 319, 680, 1442, 581, 394, 50, 1739, 1390, 1959, 744, 1722,
   640, 451, 1500, 94, 1883, 177, 947, 191, 1801, 1289, 1915, 1567, 962, 127, 245, 1777, 1544, 520, 6, 1795, 122, 290, 1363,
   1249, 320, 1183, 681, 607, 1443, 1331, 582, 1897, 395, 1832, 51, 831, 1740, 217, 1391, 357, 1960, 410, 745, 992, 1723,295,
   641, 376, 452, 543, 1501, 88, 95, 979, 1884, 108, 178, 665, 948, 1163, 192, 1652, 1802, 725, 1290, 838, 1916, 101, 1568,
   810, 963, 1863, 128, 789, 246, 1476, 1778, 1368, 1545, 29, 521, 1268, 7, 738, 1796, 1955, 123, 1327, 291, 1648, 1364, 1008,
   1250, 1012, 321, 504, 1184, 1687, 682, 210, 608, 1590, 1444, 1303, 1332, 921, 583, 437, 1898, 1254, 396, 1673, 1833, 1200,
   52, 445, 832, 1104, 1741, 1623, 218, 1847, 1392, 851, 358, 1016, 1961, 708, 411, 1110, 746, 985, 993, 1461, 1724, 2024, 296,
   1226, 642, 2016, 377, 325, 453, 1929, 544, 859, 1502, 1356, 89, 826, 96, 205, 980, 1171, 1885, 1947, 109, 508, 179, 1484,
   666, 627, 949, 1176, 1164, 615, 193, 1408, 1653, 150, 1803, 1000, 726, 1188, 1291, 1092, 839, 1214, 1917, 1436, 102, 1617,
   1569, 1575, 811, 162, 964, 913, 1864, 1691, 129, 1815, 790, 1747, 247, 1890, 1477, 1420, 1779, 268, 1369, 66, 1546,
   1665, 30, 686, 522, 486, 1269, 891, 8, 1493, 739, 577, 1797, 316, 1956, 174, 124, 287, 1328, 214, 292, 976, 1649, 807,
   1365, 1952, 1009, 1587, 1251, 1101, 1013, 1458, 322, 823, 505, 612, 1185, 1614, 1688, 1417, 683, 574, 211, 1584, 609, 1581,
   1591, 564, 1445, 1824, 1304, 1594, 1333, 428, 922, 1629, 584, 114, 438, 567, 1899, 1639, 1255, 277, 397, 1241, 1674, 1448,
   1834, 495, 1201, 1604, 53, 238, 446, 1827, 833, 1682, 1105, 622, 1742, 802, 1624, 1307, 219, 1427, 1848, 1312, 1393, 513,
   852, 1597, 359, 882, 1017, 1759, 1962, 1468, 709, 1336, 412, 259, 1111, 1144, 747, 1876, 986, 431, 994, 817, 1462, 2033,
   1725, 657, 2025, 925, 297, 1938, 1227, 224, 643, 184, 2017, 1632, 378, 141, 326, 1525, 454, 717, 1930, 587, 545, 1083, 860,
   1703, 1503, 2043, 1357, 117, 90, 1791, 827, 84, 97, 734, 206, 441, 981, 1352, 1172, 1432, 1886, 1489, 1948, 570, 110, 234,
   509, 1872, 180, 2039, 1485, 1902, 667, 38, 628, 935, 950, 1906, 1177, 1642, 1165, 168, 616, 78, 194, 1535, 1409, 1258, 1654,
   1558, 151, 1853, 1804, 671, 1001, 280, 727, 385, 1189, 597, 1292, 1381, 1093, 400, 840, 1713, 1215, 366, 1918, 42, 1437, 1244,
   103, 1322, 1618, 200, 1570, 311, 1576, 1677, 812, 1786, 163, 1317, 965, 632, 914, 1451, 1865, 534, 1692, 2006, 130, 1731,
   1816, 1837, 791, 348, 1748, 698, 248, 939, 1891, 498, 1478, 970, 1421, 1346, 1780, 1281, 269, 1204, 1370, 1154, 67, 1398,
   1547, 954, 1666, 1607, 31, 780, 687, 903, 523, 1769, 487, 56, 1270, 20, 892, 476, 9, 1494, 1910, 740, 241, 578, 637, 1798,
   1541, 317, 392, 1957, 449, 175, 1287, 125, 518, 288, 1181, 1329, 1830, 215, 408, 293, 541, 977, 663, 1650, 836, 808, 787,
   1366, 1266, 1953, 1646, 1010, 1685, 1588, 919, 1252, 1198, 1102, 1845, 1014, 1108, 1459, 1224, 323, 857, 824, 1169, 506,
   625, 613, 148, 1186, 1212, 1615, 160, 1689, 1745, 1418, 64, 684, 889, 575, 172, 212, 805, 1585, 1456, 610, 1415, 1582,
   562, 1592, 1627, 565, 275, 1446, 1602, 1825, 620, 1305, 1310, 1595, 1757, 1334, 1142, 429, 2031, 923, 222, 1630, 1523, 585,
   1701, 115, 82, 439, 1430, 568, 1870, 1900, 933, 1640, 76, 1256, 1851, 278, 595, 398, 364, 1242, 198, 1675, 1315, 1449, 2004,
   1835, 696, 496, 1344, 1202, 1396, 1605, 901, 54, 474, 239, 1539, 447, 516, 1828, 539, 834, 1264, 1683, 1196, 1106, 855, 623,
   1210, 1743, 887, 803, 1413, 1625, 1600, 1308, 1140, 220, 1699, 1428, 931, 1849, 362, 1313, 694, 1394, 472, 514, 1262, 853,
   885, 1598, 1697, 360, 470, 883, 468, 1018, 1020, 1760, 1066, 1963, 1022, 1469, 1658, 710, 1762, 1337, 333, 413, 1068, 260,
   772, 1112, 1965, 1145, 1980, 748, 1024, 1877, 1562, 987, 1471, 432, 2011, 995, 1660, 818, 1236, 1463, 712, 2034, 1376, 1726,
   1764, 658, 155, 2026, 1339, 926, 767, 298, 335, 1939, 303, 1228, 415, 225, 869, 644, 1070, 185, 1857, 2018, 262, 1633, 135,
   379, 774, 142, 1998, 327, 1114, 1526, 1127, 455, 1967, 718, 1808, 1931, 1147, 588, 1120, 546, 1982, 1084, 340, 861, 750, 1704,
   1048, 1504, 1026, 2044, 675, 1358, 1879, 118, 1736, 91, 1564, 1792, 604, 828, 989, 85, 1160, 98, 1473, 735, 1005, 207, 434,
   442, 848, 982, 2013, 1353, 1944, 1173, 997, 1433, 910, 1887, 1662, 1490, 284, 1949, 820, 571, 1821, 111, 1238, 235, 799, 510,
   1465, 1873, 654, 181, 714, 2040, 731, 1486, 2036, 1903, 1532, 668, 1378, 39, 308, 629, 1728, 936, 1278, 951, 1766, 1907, 389,
   1178, 660, 1643, 1842, 1166, 157, 169, 559, 617, 2028, 79, 73, 195, 1341, 1536, 1193, 1410, 928, 1259, 465, 1655, 769, 1559,
   1233, 152, 300, 1854, 1995, 1805, 337, 672, 601, 1002, 1941, 281, 796, 728, 305, 386, 556, 1190, 1230, 598, 553, 1293, 417,
   1382, 1296, 1094, 227, 401, 1133, 841, 871, 1714, 420, 1216, 646, 367, 1514, 1919, 1072, 43, 1385, 1438, 187, 1245, 353, 104,
   1859, 1323, 1299, 1619, 2020, 201, 1404, 1571, 264, 312, 1097, 1577, 1635, 1678, 878, 813, 137, 1787, 230, 164, 381, 1318, 530,
   966, 776, 633, 404, 915, 144, 1452, 1753, 1866, 2000, 535, 1136, 1693, 329, 2007, 763, 131, 1116, 1732, 844, 1817, 1528, 1838,
   461, 792, 1129, 349, 874, 1749, 457, 699, 1057, 249, 1969, 940, 1717, 1892, 720, 499, 703, 1479, 1810, 971, 423, 1422, 1933,
   1347, 1553, 1781, 1149, 1282, 1219, 270, 590, 1205, 1061, 1371, 1122, 1155, 649, 68, 548, 1399, 758, 1548, 1984, 955, 370, 1667,
   1086, 1608, 253, 32, 342, 781, 1517, 688, 863, 904, 1989, 524, 752, 1770, 1922, 488, 1706, 57, 1973, 1271, 1050, 21, 1075, 893,
   1506, 477, 1039, 10, 1028, 2046, 1495, 46, 1911, 677, 741, 944, 242, 1360, 579, 1388, 638, 1881, 1799, 960, 1542, 120, 318, 1441,
   393, 1738, 1958, 1721, 450, 93, 176, 190, 1288, 1566, 126, 1776, 519, 1794, 289, 1248, 1182, 606, 1330, 1896, 1831, 830, 216,
   356, 409, 991, 294, 375, 542, 87, 978, 107, 664, 1162, 1651, 724, 837, 100, 809, 1862, 788, 1475, 1367, 28, 1267, 737, 1954, 1326,
   1647, 1007, 1011, 503, 1686, 209, 1589, 1302, 920, 436, 1253, 1672, 1199, 444, 1103, 1622, 1846, 850, 1015, 707, 1109, 984, 1460,
   2023, 1225, 2015, 324, 1928, 858, 1355, 825, 204, 1170, 1946, 507, 1483, 626, 1175, 614, 1407, 149, 999, 1187, 1091, 1213, 1435,
   1616, 1574, 161, 912, 1690, 1814, 1746, 1889, 1419, 267, 65, 1664, 685, 485, 890, 1492, 576, 315, 173, 286, 213, 975, 806, 1951,
   1586, 1100, 1457, 822, 611, 1613, 1416, 573, 1583, 1580, 563, 1823, 1593, 427, 1628, 113, 566, 1638, 276, 1240, 1447, 494, 1603,
   237, 1826, 1681, 621, 801, 1306, 1426, 1311, 512, 1596, 881, 1758, 1467, 1335, 258, 1143, 1875, 430, 816, 2032, 656, 924, 1937,
   223, 183, 1631, 140, 1524, 716, 586, 1082, 1702, 2042, 116, 1790, 83, 733, 440, 1351, 1431, 1488, 569, 233, 1871, 2038, 1901,
   37, 934, 1905, 1641, 167, 77, 1534, 1257, 1557, 1852, 670, 279, 384, 596, 1380, 399, 1712, 365, 41, 1243, 1321, 199, 310,
   1676, 1785, 1316, 631, 1450, 533, 2005, 1730, 1836, 347, 697, 938, 497, 969, 1345, 1280, 1203, 1153, 1397, 953, 1606, 779,
   902, 1768, 55, 19, 475, 1909, 240, 636, 1540, 391, 448, 1286, 517, 1180, 1829, 407, 540, 662, 835, 786, 1265, 1645, 1684,
   918, 1197, 1844, 1107, 1223, 856, 1168, 624, 147, 1211, 159, 1744, 63, 888, 171, 804, 1455, 1414, 561, 1626, 274, 1601, 619,
   1309, 1756, 1141, 2030, 221, 1522, 1700, 81, 1429, 1869, 932, 75, 1850, 594, 363, 197, 1314, 2003, 695, 1343, 1395, 900, 473,
   1538, 515, 538, 1263, 1195, 854, 1209, 886, 1412, 1599, 1139, 1698, 930, 361, 693, 471, 1261, 884, 1696, 469, 467, 1019, 1065,
   1021, 1657, 1761, 332, 1067, 771, 1964, 1979, 1023, 1561, 1470, 2010, 1659, 1235, 711, 1375, 1763, 154, 1338, 766, 334, 302, 414,
   868, 1069, 1856, 261, 134, 773, 1997, 1113, 1126, 1966, 1807, 1146, 1119, 1981, 339, 749, 1047, 1025, 674, 1878, 1735, 1563, 603,
   988, 1159, 1472, 1004, 433, 847, 2012, 1943, 996, 909, 1661, 283, 819, 1820, 1237, 798, 1464, 653, 713, 730, 2035, 1531, 1377, 307,
   1727, 1277, 1765, 388, 659, 1841, 156, 558, 2027, 72, 1340, 1192, 927, 464, 768, 1232, 299, 1994, 336, 600, 1940, 795, 304, 555, 1229,
   552, 416, 1295, 226, 1132, 870, 419, 645, 1513, 1071, 1384, 186, 352, 1858, 1298, 2019, 1403, 263, 1096, 1634, 877, 136, 229, 380,
   529, 775, 403, 143, 1752, 1999, 1135, 328, 762, 1115, 843, 1527, 460, 1128, 873, 456, 1056, 1968, 1716, 719, 702, 1809, 422, 1932,
   1552, 1148, 1218, 589, 1060, 1121, 648, 547, 757, 1983, 369, 1085, 252, 341, 1516, 862, 1988, 751, 1921, 1705, 1972, 1049, 1074, 1505,
   1038, 1027, 2045, 45, 676, 943, 1359, 1387, 1880, 959, 119, 1440, 1737, 1720, 92, 189, 1565, 1775, 1793, 1247, 605, 1895, 829, 355,
   990, 374, 86, 106, 1161, 723, 99, 1861, 1474, 27, 736, 1325, 1006, 502, 208, 1301, 435, 1671, 443, 1621, 849, 706, 983, 2022, 2014,
   1927, 1354, 203, 1945, 1482, 1174, 1406, 998, 1090, 1434, 1573, 911, 1813, 1888, 266, 1663, 484, 1491, 314, 285, 974, 1950, 1099, 821,
   1612, 572, 1579, 1822, 426, 112, 1637, 1239, 493, 236, 1680, 800, 1425, 511, 880, 1466, 257, 1874, 815, 655, 1936, 182, 139, 715, 1081,
   2041, 1789, 732, 1350, 1487, 232, 2037, 36, 1904, 166, 1533, 1556, 669, 383, 1379, 1711, 40, 1320, 309, 1784, 630, 532, 1729, 346, 937,
   968, 1279, 1152, 952, 778, 1767, 18, 1908, 635, 390, 1285, 1179, 406, 661, 785, 1644, 917, 1843, 1222, 1167, 146, 158, 62, 170, 1454,
   560, 273, 618, 1755, 2029, 1521, 80, 1868, 74, 593, 196, 2002, 1342, 899, 1537, 537, 1194, 1208, 1411, 1138, 929, 692, 1260, 1695, 466,
   1064, 1656, 331, 770, 1978, 1560, 2009, 1234, 1374, 153, 765, 301, 867, 1855, 133, 1996, 1125, 1806, 1118, 338, 1046, 673, 1734, 602,
   1158, 1003, 846, 1942, 908, 282, 1819, 797, 652, 729, 1530, 306, 1276, 387, 1840, 557, 71, 1191, 463, 1231, 1993, 599, 794, 554, 551,
   1294, 1131, 418, 1512, 1383, 351, 1297, 1402,1095, 876, 228, 528, 402, 1751, 1134, 761, 842, 459, 872, 1055, 1715, 701, 421, 1551,
   1217, 1059, 647, 756, 368, 251, 1515, 1987, 1920, 1971, 1073, 1037, 44, 942, 1386, 958, 1439, 1719, 188, 1774, 1246, 1894, 354, 373,
   105, 722, 1860, 26, 1324, 501, 1300, 1670, 1620, 705, 2021, 1926, 202, 1481, 1405, 1089, 1572, 1812, 265, 483, 313, 973, 1098, 1611,
   1578, 425, 1636, 492, 1679, 1424, 879, 256, 814, 1935, 138, 1080, 1788, 1349, 231, 35, 165, 1555, 382, 1710, 1319, 1783, 531, 345,
   967, 1151, 777, 17, 634, 1284, 405, 784, 916, 1221, 145, 61, 1453, 272, 1754, 1520, 1867, 592, 2001, 898, 536, 1207, 1137, 691, 1694,
   1063, 330, 1977, 2008, 1373, 764, 866, 132, 1124, 1117, 1045, 1733, 1157, 845, 907, 1818, 651, 1529, 1275, 1839, 70, 462, 1992, 793,
   550, 1130, 1511, 350, 1401, 875, 527, 1750, 760, 458, 1054, 700, 1550, 1058, 755, 250, 1986, 1970, 1036, 941, 957, 1718, 1773, 1893,
   372, 721, 25, 500, 1669, 704, 1925, 1480, 1088, 1811, 482, 972, 1610, 424, 491, 1423, 255, 1934, 1079, 1348, 34, 1554, 1709, 1782,
   344, 1150, 16, 1283, 783, 1220, 60, 271, 1519, 591, 897, 1206, 690, 1062, 1976, 1372, 865, 1123, 1044, 1156, 906, 650, 1274, 69, 1991,
   549, 1510, 1400, 526, 759, 1053, 1549, 754, 1985, 1035, 956, 1772, 371, 24, 1668, 1924, 1087, 481, 1609, 490, 254, 1078, 33, 1708,
   343, 15, 782, 59, 1518, 896, 689, 1975, 864, 1043, 905, 1273, 1990, 1509, 525, 1052, 753, 1034, 1771, 23, 1923, 480, 489, 1077, 1707,
   14, 58, 895, 1974, 1042, 1272, 1508, 1051, 1033, 22, 479, 1076, 13, 894, 1041, 1507, 1032, 478, 12, 1040, 1031, 11, 1030, 1029, 1024
};
    constexpr unsigned char xijishu[8] = { 2,5,1,6,3,4,0,7 };

    unsigned short rw_probe_decodecomp(unsigned char* buffer, int pos) {
        unsigned short temp = 0;
        for (int i = 0; i < 14; i++) {
            temp |= ((buffer[i] >> (pos % 8)) & 0x1) << (13 - i);
        }
        return temp;
    }

    void rw_probe_datarecombine(unsigned short* data) {
        for (int i = 0; i < 4096; i++) {
            unsigned short caiji = data[i] & 0x7FF;    // 低11位
            unsigned char xiji = (data[i] >> 11) & 0x7; // 高3位
            data[i] = (cujishu[caiji] << 3) | xijishu[xiji];
        }
    }

    void rw_probe_decode(const unsigned char* buffer, unsigned short* outdata)
    {
        int i, j, k;
        int i_div8, base;
        unsigned char tempbuffer[14] = { 0 };
        //第一个 3584byte
        for (i = 0; i < 32; i++)
        {
            i_div8 = i >> 3;
            for (j = 0; j < 64; j++)
            {
                base = i_div8 + 56 * j;
                for (k = 0; k < 14; k++)//第k位
                {
                    tempbuffer[k] = buffer[base + (k << 2)];
                }
                // for(n=0;n<14;n++)
                // {
                //     printf("0x%x  ",tempbuffer[n]);
                // }
                // printf("\r\n");
                outdata[128 * i + j] = rw_probe_decodecomp(tempbuffer, i);
            }
        }
        //第二个 3584byte
        for (i = 0; i < 32; i++)
        {
            i_div8 = i >> 3;
            for (j = 0; j < 64; j++)
            {
                base = 3584 + i_div8 + 56 * j;
                for (k = 0; k < 14; k++)
                {
                    tempbuffer[k] = buffer[base + (k << 2)];
                }
                outdata[128 * i + 64 + j] = rw_probe_decodecomp(tempbuffer, i);
            }
        }
        rw_probe_datarecombine(outdata);
        return;
    }

    void reorder_to_colmajor(const unsigned short* data_32x128, unsigned short* data_colmajor_64x64){
        // data_32x128: 长度4096， 当前布局32x128 行优先
        // data_colmajor_64x64: 长度4096， 目标布局64x64 列优先
        // 将32x128数据重新排列为64x64列主序 （0-64列为上半部分，65-128列为下半部分）
        for (int r = 0; r < 32; r++){
            // 左半 (上半行32行)
            for (int c = 0; c < 64; c++){
                int R = r;
                int C = c;
                int src_idx = r * 128 + c;
                int dst_idx = C * 64 + R;
                data_colmajor_64x64[dst_idx] =  data_32x128[src_idx];
            }

            // 右半 (下半行32行)
            for (int c = 64; c < 128; c++){
                int R = r + 32;
                int C = c - 64;
                int src_idx = r * 128 + c;
                int dst_idx = C * 64 + R;
                data_colmajor_64x64[dst_idx] =  data_32x128[src_idx];
            }
        }
    }

    void flip180_colmajor64x64(const unsigned short* data_colmajor_64x64, unsigned short* data_colmajor_64x64_flipped){
        // data_colmajor_64x64: 长度4096， 当前布局64x64 列优先
        // data_colmajor_64x64_flipped: 长度4096， 目标布局64x64 列优先
        // 将64x64列主序数据进行180度翻转
        for (int c = 0; c < 64; c++){
            for (int r = 0; r < 64; r++){
                int src_idx = c * 64 + r;
                int dst_idx = (63 - c) * 64 + (63 - r);
                data_colmajor_64x64_flipped[dst_idx] = data_colmajor_64x64[src_idx];
            }
        }
    }

    void rw_probe_decode3(const unsigned char* buffer, unsigned short* outdata)
    {
        /*
        第一个 3584: [64 * i + j] -> [64 * (63 - i) + j]
        第二个 3584: [64 * (i + 32) + j] -> [64 * (31 - i) + j]
        这是 行优先存储的结果 -> T(转置) -> 列优先存储的矩阵 -> flipud -> 列优先存储（上下翻转的矩阵） -> T -> 行优先存储（左右翻转的矩阵）
        等价于 行优先存储 -> fliplr -> 行优先存储（左右翻转的矩阵）
        */
        int i, j, k;
        int i_div8, base;
        unsigned char tempbuffer[14] = { 0 };
        //第一个 3584byte
        for (i = 0; i < 32; i++) // i 代表行
        {
            i_div8 = i >> 3;
            for (j = 0; j < 64; j++) // j 代表列
            {
                base = i_div8 + 56 * (64 - j - 1); // 列反转
                for (k = 0; k < 14; k++)//第k位
                {
                    tempbuffer[k] = buffer[base + (k << 2)];
                }
                outdata[64 * i + j] = rw_probe_decodecomp(tempbuffer, i); // 参照0929.md
            }
        }

        //第二个 3584byte
        for (i = 0; i < 32; i++) // i 代表行
        {
            i_div8 = i >> 3;
            for (j = 0; j < 64; j++) // j 代表列
            {
                base = 3584 + i_div8 + 56 * (64 - j - 1); // 列反转
                for (k = 0; k < 14; k++)
                {
                    tempbuffer[k] = buffer[base + (k << 2)];
                }
                outdata[64 * (i + 32) + j] = rw_probe_decodecomp(tempbuffer, i); // 参照0929.md
            }
        }
        rw_probe_datarecombine(outdata);
    }
    
    void rotate180_rowmajor(unsigned short decData[4096]) {
        const int ROWS = 64;
        const int total = ROWS * ROWS;
        for (int i = 0; i < total / 2; i++) {
            std::swap(decData[i], decData[total - 1 - i]);
        }
    }
}