
#include "jaka_kinematics.h"

namespace jaka_kinematics
{

    double a2 = 0.429733;
    double a3 = 0.368162;
    double d1 = 0.1211;
    double d4 = -0.114195;
    double d5 = 0.113416;
    double d6 = 0.107119;

    int SIGN(double x)
    {
        return (x > 0) - (x < 0);
    }
    double rotate(double t)
    {
        if (t > 2 * M_PI)
            t -= 2 * M_PI;
        else if (t < 0)
            t += 2 * M_PI;
        return t;
    }
    void to_mat44(double *T, double *trans, double *rot)
    {
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; j++)
                *(T++) = *(rot++);
            *(T++) = *(trans++);
        }
        *(T++) = 0;
        *(T++) = 0;
        *(T++) = 0;
        *(T++) = 1;
    }

    void from_mat44(const double *T, double *trans, double *rot)
    {
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; j++)
                *(rot++) = *(T++);
            *(trans++) = *(T++);
        }
    }
    
    void from_rpy(double *rot, double *rpy)
    {
        double rx = *(rpy++);
        double ry = *(rpy++);
        double rz = *(rpy++);
        *(rot++) = cos(rz) * cos(ry);
        *(rot++) = cos(rz) * sin(ry) * sin(rx) - sin(rz) * cos(rx);
        *(rot++) = cos(rz) * sin(ry) * cos(rx) + sin(rz) * sin(rx);

        *(rot++) = sin(rz) * cos(ry);
        *(rot++) = sin(rz) * sin(ry) * sin(rx) + cos(rz) * cos(rx);
        *(rot++) = sin(rz) * sin(ry) * cos(rx) - cos(rz) * sin(rx);

        *(rot++) = -sin(ry);
        *(rot++) = cos(ry) * sin(rx);
        *(rot++) = cos(ry) * cos(rx);
    }

    void forward(double *q, double *T)
    {
        double s1 = sin(*q), c1 = cos(*q);
        q++;
        double tmp = *q, s2 = sin(*q), c2 = cos(*q);
        q++;
        double s3 = sin(*q), c3 = cos(*q);
        tmp += *q;
        q++;
        double s23 = sin(tmp), c23 = cos(tmp);
        tmp += *q;
        q++;
        double s5 = sin(*q), c5 = cos(*q);
        q++;
        double s6 = sin(*q), c6 = cos(*q);
        double s234 = sin(tmp), c234 = cos(tmp);

        *(T++) = (s1 * s5 + c1 * c5 * c234) * c6 - s6 * s234 * c1;
        *(T++) = -(s1 * s5 + c1 * c5 * c234) * s6 - s234 * c1 * c6;
        *(T++) = s1 * c5 - s5 * c1 * c234;
        *(T++) = a2 * c1 * c2 + a3 * c1 * c23 + d4 * s1 + d5 * s234 * c1 + d6 * (s1 * c5 - s5 * c1 * c234);
        *(T++) = (s1 * c5 * c234 - s5 * c1) * c6 - s1 * s6 * s234;
        *(T++) = -(s1 * c5 * c234 - s5 * c1) * s6 - s1 * s234 * c6;
        *(T++) = -s1 * s5 * c234 - c1 * c5;
        *(T++) = a2 * s1 * c2 + a3 * s1 * c23 - d4 * c1 + d5 * s1 * s234 - d6 * (s1 * s5 * c234 + c1 * c5);
        *(T++) = s6 * c234 + s234 * c5 * c6;
        *(T++) = -s6 * s234 * c5 + c6 * c234;
        *(T++) = -s5 * s234;
        *(T++) = a2 * s2 + a3 * s23 + d1 - d5 * c234 - d6 * s5 * s234;

        *(T++) = 0;
        *(T++) = 0;
        *(T++) = 0;
        *(T++) = 1;
    }
    
    int inverse(double *matrix, double *solution, double q6_des)
    {
        int num_sols = 0;
        double nx, ny, nz, oz, ox, oy, ax, ay, az, pz, px, py;
        double *p = matrix;
        nx = *p;
        p++;
        ox = *p;
        p++;
        ax = *p;
        p++;
        px = *p;
        p++;

        ny = *p;
        p++;
        oy = *p;
        p++;
        ay = *p;
        p++;
        py = *p;
        p++;

        nz = *p;
        p++;
        oz = *p;
        p++;
        az = *p;
        p++;
        pz = *p;
        // J1
        double q1[2];
        {
            // d₆*(-ax*s1 + ay*c1) + px*s1 + py*c1 = d4

            // -d4 + d6*(-ax*s1 + ay*c1) + px*s1 - py*c1 = 0
            // ==> (d6*ay - py)c1 + (px - d6*ax)s1 -d4 = 0
            // ==>A = d6*ay - py; B = px - d6*ax; C = -d4;
            // 借助以下结论求解：
            // Acosx + Bsinx + C = 0
            // 构造辅助角 phi
            // cos(phi) = \frac{A}{sqrt(A*A+B*B)}; sin(phi)=\frac{B}{sqrt(A*A+B*B)}; tan(phi) = \frac{B}{A}
            // Acosx + Bsinx + C = 0  ==> cos(x-phi)+\frac{C}{sqrt(A*A+B*B)} = 0 ==> x-phi= arccos(\frac{-C}{sqrt(A*A+B*B)})
            // Acosx + Bsinx + C = 0  也可以写成 cos(phi-x)+\frac{C}{sqrt(A*A+B*B)} = 0 后面结果无影响，
            double B = -(d6 * ax - px);
            double A = d6 * ay - py;
            double R = A * A + B * B;
            if (fabs(A) < ZERO_THRESH)
            {
                double div;
                if (fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
                    div = -SIGN(d4) * SIGN(B);
                else
                    div = d4 / B;
                double arcsin = asin(div);
                if (fabs(arcsin) < ZERO_THRESH)
                    arcsin = 0.0;
                if (arcsin < 0.0)
                    q1[0] = arcsin + 2.0 * PI;
                else
                    q1[0] = arcsin;
                q1[1] = PI - arcsin;
            }
            else if (fabs(B) < ZERO_THRESH)
            {
                double div;
                if (fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
                    div = SIGN(d4) * SIGN(A);
                else
                    div = d4 / A;
                double arccos = acos(div);
                q1[0] = arccos;
                q1[1] = 2.0 * PI - arccos;
            }
            else if (d4 * d4 > R)
                return num_sols;
            else
            {
                // arccos 是 x - phi的值; arctan 是 phi 的值
                // double arccos = acos(-C / sqrt(R)) ;
                double arccos = acos(d4 / sqrt(R));
                double arctan = atan2(B, A);
                // pos 是 x， 是需要求的值; neg 是 2*phi - x 代入原方程式是成立的
                double pos = arctan + arccos;
                double neg = arctan - arccos;
                if (fabs(pos) < ZERO_THRESH)
                    pos = 0.0;
                if (fabs(neg) < ZERO_THRESH)
                    neg = 0.0;
                if (pos >= 0.0)
                    q1[0] = pos;
                else
                    q1[0] = 2.0 * PI + pos;
                if (neg >= 0.0)
                    q1[1] = neg;
                else
                    q1[1] = 2.0 * PI + neg;
            }
        }
        double q5[2][2];
        {
            for (int i = 0; i < 2; i++)
            {
                // ax*s1 - ay*c1 - c5 = 0
                double numer = (ax * sin(q1[i]) - ay * cos(q1[i]));
                double arccos = acos(numer);
                q5[i][0] = arccos;
                q5[i][1] = 2.0 * PI - arccos;
            }
        }
        {
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {

                    double c1 = cos(q1[i]), s1 = sin(q1[i]);
                    double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
                    double q6;
                    ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
                    if (fabs(s5) < ZERO_THRESH)
                        q6 = q6_des;
                    else
                    {
                        // (-nx*s1 + ny*c1)*s6 + (-ox*s1 + oy*c1)*c6 = 0
                        // (-nx*s1 + ny*c1)*c6 - (-ox*s1 + oy*c1)*s6 = -s5
                        // 这两个联合求解
                        // (-nx*s1 + ny*c1)*c6 - (-ox*s1 + oy*c1)*s6 = -s5

                        q6 = atan2(SIGN(s5) * -(ox * s1 - oy * c1),
                                   SIGN(s5) * (nx * s1 - ny * c1));
                        if (fabs(q6) < ZERO_THRESH)
                            q6 = 0.0;
                        if (q6 < 0.0)
                            q6 += 2.0 * PI;
                    }
                    double q2[2], q3[2], q4[2];
                    ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
                    double c6 = cos(q6), s6 = sin(q6);
                    // J3
                    // a3*c2 + a4*(-s2*sin(theta3) + c2*cos(theta3)) = 0 -d5*(-(nx*c1 + ny*s1)*s6 - (ox*c1 + oy*s1)*c6) - d6*(ax*c1 + ay*s1) + px*c1 + py*s1
                    double p13x = d5 * ((nx * c1 + ny * s1) * s6 + (ox * c1 + oy * s1) * c6) - d6 * (ax * c1 + ay * s1) + px * c1 + py * s1;
                    double p13y = -az * d6 - d1 + d5 * (nz * s6 + oz * c6) + pz;
                    double c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3);
                    if (fabs(fabs(c3) - 1.0) < ZERO_THRESH)
                        c3 = SIGN(c3);
                    else if (fabs(c3) > 1.0)
                    {
                        continue; // TODO NO SOLUTION
                    }
                    double arccos = acos(c3);
                    q3[0] = arccos;
                    q3[1] = 2.0 * PI - arccos;
                    // J2
                    double s3 = sin(arccos);
                    double A = (a2 + a3 * c3), B = a3 * s3;
                    double p04x = -(ax * c1 + ay * s1) * s5 + ((nx * c1 + ny * s1) * c6 - (ox * c1 + oy * s1) * s6) * c5;
                    double p04y = (nx * c1 + ny * s1) * s6 + (ox * c1 + oy * s1) * c6;
                    q2[0] = atan2((A * p13y - B * p13x), (A * p13x + B * p13y));
                    q2[1] = atan2((A * p13y + B * p13x), (A * p13x - B * p13y));

                    // J5 = J234 -J2 -J3
                    q4[0] = atan2(-p04y, p04x) - q2[0] - q3[0];
                    q4[1] = atan2(-p04y, p04x) - q2[1] - q3[1];
                    ////////////////////////////////////////////////////////////////////////////////
                    for (int k = 0; k < 2; k++)
                    {
                        if (fabs(q2[k]) < ZERO_THRESH)
                            q2[k] = 0.0;
                        else if (q2[k] < 0.0)
                            q2[k] += 2.0 * PI;
                        if (fabs(q4[k]) < ZERO_THRESH)
                            q4[k] = 0.0;
                        else if (q4[k] < 0.0)
                            q4[k] += 2.0 * PI;
                        solution[num_sols * 6 + 0] = q1[i];
                        solution[num_sols * 6 + 1] = q2[k];
                        solution[num_sols * 6 + 2] = q3[k];
                        solution[num_sols * 6 + 3] = q4[k];
                        solution[num_sols * 6 + 4] = q5[i][j];
                        solution[num_sols * 6 + 5] = q6;
                        num_sols++;
                    }
                }
            }
        }
        for (int i = 0; i < num_sols; i++)
        {
            for (int j = 0; j < 6; j++)
                solution[6 * i + j] = rotate(solution[6 * i + j]);
        }
        return num_sols;
    }
}
