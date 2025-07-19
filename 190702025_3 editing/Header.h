#pragma once
#define _USE_MATH_DEFINES
// === Libraries ===
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


// === Standard Libraries ===
#include <iostream>.
#include <fstream>
#include <vector>
#include <set>
#include <unordered_set>
#include <map>
#include <limits>
#include <cmath>

// ===  Eigen + libigl Poisson solve ===
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <igl/grad.h>
//#include <igl/div.h>
#include <igl/cotmatrix.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/connected_components.h>


//CGAL
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Surface_mesh_deformation.h>
#include <CGAL/IO/STL/STL_reader.h>
#include <CGAL/Polygon_mesh_processing/smooth_shape.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/IO/STL.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/Timer.h>

#include <boost/lexical_cast.hpp>
#include <assert.h>



//// struct & Enum
struct Vertex {
    float x, y, z;
};
//Global Variables
//Model & Deformation Data
std::set<int> handle_indices, fixed_indices, smooth_indices;

//Selection State
bool selecting_handle = false, selecting_fixed = false, selecting_smooth = false, smooth_select_region = false, selection_done = false;
bool handle_select_vertices = false, handle_select_region = false;// Nested options
bool fixed_select_vertices = false, fixed_select_region = false;
bool show_handles = true, show_fixed = true, show_region = true;

//
std::vector<ImVec2> region_polygon; // Region polygon (2D screen space)
std::vector<glm::vec2> selectionPolygon;
bool drawing_region = false, region_selection_done = false, selecting_handle_region = false, selecting_fixed_region = false;// Selection state

// Handle Manipulation
bool mouseControlledDeform = false;
bool dragging_handle = false;
int dragged_handle = -1;
float tx = 0.0f, ty = 0.0f, tz = 0.0f;


//rotation
bool rotation_enabled = false;
bool boundingBoxVisible = false;
glm::vec3 bboxCenter(0.0f);
glm::vec3 bboxMin, bboxMax;
float rotationAngleY = 0.0f;
float rotationAngleX = 0.0f;
float rotationAngleZ = 0.0f;


//smooth
bool smoothSelectedMeshArea = false;


//Camera
enum CameraProjectionMode { PERSPECTIVE, ORTHOGRAPHIC };
enum CameraControlMode { ORBIT, FREE, MODEL_VIEW };
enum RenderMode { SHADED, WIREFRAME, BOTH, VERTICES };
float modelScale = 1.0f;
glm::vec3 modelOffset(0.0f);
float modelRotationAngle = 0.0f;
glm::vec3 modelRotationAxis(0.0f, 1.0f, 0.0f);
float cameraDistance = 5.0f;
float cameraYaw = 0.0f, cameraPitch = 0.0f;
float lastX = 0.0f, lastY = 0.0f;
bool rotating = false;
glm::vec3 cameraTarget(0.0f);
CameraProjectionMode cameraMode = PERSPECTIVE;
CameraControlMode controlMode = ORBIT;
RenderMode renderMode;
static bool showCameraWindow = false;


// mainwindow
GLFWwindow* window = nullptr;
bool loaded = false;


std::vector<std::array<int, 3>> triangleIndices;  // Indices for each triangle in the mesh
bool is_dragging = false;
bool mouse_released = false;
glm::vec3 drag_start_pos;

//cgal
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3> Polyhedron;
typedef boost::graph_traits<Polyhedron>::vertex_descriptor vertex_descriptor;
typedef CGAL::Surface_mesh_deformation<Polyhedron, CGAL::Default, CGAL::Default, CGAL::SRE_ARAP> Surface_mesh_deformation;
Polyhedron mesh, demesh;
std::unique_ptr<Surface_mesh_deformation> deform, deformed_model;
std::unordered_map<int, Kernel::Point_3> original_positions;
using Vertex_iterator = Polyhedron::Vertex_iterator;
typedef Polyhedron::Facet_iterator Facet_iterator;
typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;
typedef Polyhedron::Facet_const_iterator Facet_const_iterator;
typedef Polyhedron::Halfedge_around_facet_const_circulator HFCC;
typedef Polyhedron::Halfedge_around_vertex_circulator      HV_circulator;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PI2         M_PI*2
#define PI          M_PI
#define PI_2        M_PI/2 
#define EPSILON        1e-8

class Point2D
{
public:
    double    m_x, m_y;
public:
    //构造函数
    Point2D(double newx = 0.0, double newy = 0.0);
};

class Point3D
{
public:
    double    m_x, m_y, m_z;
public:
    //构造函数
    Point3D(double newx = 0.0, double newy = 0.0, double newz = 0.0);
    void operator=(Point3D point);
    Point3D& operator*=(double num);
    Point3D operator*(double num);
    Point3D operator+(Point3D point);
    void operator/=(int num);
    Point3D& operator += (const Point3D& p);
    Point3D& operator-=(Point3D point);
};

class Vector2D
{
public:
    double    m_x, m_y;

public:
    Vector2D(double newx = 0.0, double newy = 0.0);

    // 赋值操作
    Vector2D& operator += (const Vector2D& v);
    Vector2D& operator -= (const Vector2D& v);
    Vector2D& operator *= (double num);
    Vector2D& operator /= (double num);
    double operator ^(const Vector2D& v);

    //单目减
    Vector2D operator - () const;

    double mf_getLength() const; // 取长度
    Vector2D mf_getPerpendicularVector() const; //得到一个垂直的向量

    void mf_normalize(); // 单位化
    void mf_setValue(double newx = 0.0, double newy = 0.0);
};

class Vector3D
{
public:
    double    m_x, m_y, m_z;

public:
    Vector3D(double newx = 0.0, double newy = 0.0, double newz = 0.0);
    Vector3D(const Point3D start, const Point3D end);

    //赋值操作
    Vector3D& operator += (const Vector3D& v);
    Vector3D& operator -= (const Vector3D& v);
    Vector3D& operator *= (double num);
    Vector3D& operator /= (double num);
    Vector3D& operator ^= (const Vector3D& v);
    bool operator == (const Vector3D& v);

    //单目减
    Vector3D operator - () const;

    double mf_getLength() const; // 取长度
    Vector3D mf_getPerpendicularVector() const; //得到一个垂直的向量

    void mf_normalize(); // 单位化
    void mf_setValue(double newx = 0.0, double newy = 0.0, double newz = 0.0);
};

//face保存顶点的索引
class Face
{
public:
    int v0;
    int v1;
    int v2;
public:
    Face(int m0, int m1, int m2);
};

extern Point2D operator + (const Point2D& pt, const Vector2D& v);
extern Point2D operator - (const Point2D& pt, const Vector2D& v);
extern Vector2D operator - (const Point2D& p, const Point2D& q);

extern Point3D operator + (const Point3D& pt, const Vector3D& v);
extern Point3D operator - (const Point3D& pt, const Vector3D& v);
extern Vector3D operator - (const Point3D& p, const Point3D& q);

extern Vector2D operator + (const Vector2D& u, const Vector2D& v);
extern Vector2D operator - (const Vector2D& u, const Vector2D& v);
extern double  operator * (const Vector2D& u, const Vector2D& v); // 点积
extern Vector2D operator * (const Vector2D& v, double num);
extern Vector2D operator / (const Vector2D& v, double num);

extern Vector3D operator + (const Vector3D& u, const Vector3D& v);
extern Vector3D operator - (const Vector3D& u, const Vector3D& v);
extern double operator * (const Vector3D& u, const Vector3D& v); // 点积
extern Vector3D operator ^ (const Vector3D& u, const Vector3D& v); // 叉积
extern Vector3D operator * (const Vector3D& v, double num);
extern Vector3D operator / (const Vector3D& v, double num);
extern double  GetDistance(const Point3D p1, const Point3D p2);
extern Vector3D& NormalizeVector3D(Vector3D& n);

double SinValue(const Vector3D& a, const Vector3D& b, const Vector3D& c);
double CosValue(const Vector3D& a, const Vector3D& b, const Vector3D& c);
double CotValue(const Vector3D& a, const Vector3D& b, const Vector3D& c);
double MaxValue(double value1, double value2);


// ////////////////////////////////////////////////////////////////////////////
// 实现类Point2D开始
Point2D::Point2D(double newx, double newy) :m_x(newx), m_y(newy)
{
} // 类Point2D构造函数结束
// 实现类Point2D结束
// ////////////////////////////////////////////////////////////////////////////

// ////////////////////////////////////////////////////////////////////////////
// 实现类Point3D开始
void Point3D::operator=(Point3D point)
{
    m_x = point.m_x;
    m_y = point.m_y;
    m_z = point.m_z;
}

void Point3D::operator/=(int num)
{
    m_x /= (double)num;
    m_y /= (double)num;
    m_z /= (double)num;
}
Point3D& Point3D::operator += (const Point3D& p)
{
    m_x += p.m_x;
    m_y += p.m_y;
    m_z += p.m_z;

    return *this;
}

Point3D Point3D::operator+(Point3D point)
{
    Point3D p;
    p.m_x = m_x + point.m_x;
    p.m_y = m_y + point.m_y;
    p.m_z = m_z + point.m_z;

    return p;
}

Point3D& Point3D::operator-=(Point3D point)
{
    m_x -= point.m_x;
    m_y -= point.m_y;
    m_z -= point.m_z;

    return *this;
}

Point3D& Point3D::operator*=(double num)
{
    m_x *= num;
    m_y *= num;
    m_z *= num;

    return *this;
}

Point3D Point3D::operator*(double num)
{
    Point3D ret;
    ret.m_x = m_x * num;
    ret.m_y = m_y * num;
    ret.m_z = m_z * num;

    return ret;
}

Point3D::Point3D(double newx, double newy, double newz) :m_x(newx), m_y(newy), m_z(newz)
{
} // 类Point3D构造函数结束
// 实现类Point3D结束
// ////////////////////////////////////////////////////////////////////////////

// ////////////////////////////////////////////////////////////////////////////
// 实现类Point3D开始
Vector2D::Vector2D(double newx, double newy) :m_x(newx), m_y(newy)
{
} // 类Vector2D构造函数结束

Vector2D& Vector2D::operator += (const Vector2D& v)
{
    m_x += v.m_x;
    m_y += v.m_y;
    return *this;
} //成员函数operator +=结束

Vector2D& Vector2D::operator -= (const Vector2D& v)
{
    m_x -= v.m_x;
    m_y -= v.m_y;
    return *this;
} //成员函数operator -=结束

Vector2D& Vector2D::operator *= (double num)
{
    m_x *= num;
    m_y *= num;
    return *this;
} //成员函数operator *=结束

Vector2D& Vector2D::operator /= (double num)
{
    m_x /= num;  // 注意这里没有处理除数为0的情形
    m_y /= num;
    return *this;
} //成员函数operator /=结束

double Vector2D::operator ^(const Vector2D& v)
{
    return(m_x * v.m_y - m_y * v.m_x);
} //成员函数operator ^结束

Vector2D Vector2D::operator - () const
{
    return Vector2D(-m_x, -m_y);
} //成员函数operator -结束

double Vector2D::mf_getLength()  const
{
    return sqrt(m_x * m_x + m_y * m_y);
} //成员函数mf_getLength结束

Vector2D Vector2D::mf_getPerpendicularVector() const
{
    return Vector2D(-m_y, m_x);
} //成员函数mf_getPerpendicularVector结束

void Vector2D::mf_normalize()
{
    double a = mf_getLength();
    (*this) /= a; // 注意: 这里没有处理当长度为0的情况
} //成员函数mf_normalize结束

void Vector2D::mf_setValue(double newx, double newy)
{
    m_x = newx;
    m_y = newy;
} //成员函数mf_setValue结束

// 实现类Vector2D结束
// ////////////////////////////////////////////////////////////////////////////


// ////////////////////////////////////////////////////////////////////////////
// 实现类Vector3D开始
Vector3D::Vector3D(double newx, double newy, double newz) :m_x(newx), m_y(newy), m_z(newz)
{
} // 类Vector3D构造函数结束

Vector3D::Vector3D(const Point3D start, const Point3D end)
{
    m_x = end.m_x - start.m_x;
    m_y = end.m_y - start.m_y;
    m_z = end.m_z - start.m_z;
}

Vector3D& Vector3D::operator += (const Vector3D& v)
{
    m_x += v.m_x;
    m_y += v.m_y;
    m_z += v.m_z;
    return *this;
} //成员函数operator +=结束

Vector3D& Vector3D::operator -= (const Vector3D& v)
{
    m_x -= v.m_x;
    m_y -= v.m_y;
    m_z -= v.m_z;
    return *this;
} //成员函数operator -=结束

Vector3D& Vector3D::operator *= (double num)
{
    m_x *= num;
    m_y *= num;
    m_z *= num;
    return *this;
} //成员函数operator *=结束

Vector3D& Vector3D::operator /= (double num)
{
    num = 1.0 / num;
    m_x *= num;
    m_y *= num;
    m_z *= num;
    return *this;
} //成员函数operator /=结束

Vector3D& Vector3D::operator ^= (const Vector3D& v)
{
    double a = m_y * v.m_z - m_z * v.m_y;
    double b = -m_x * v.m_z + m_z * v.m_x;
    double c = m_x * v.m_y - m_y * v.m_x;

    m_x = a;
    m_y = b;
    m_z = c;
    return *this;
} //成员函数operator ^=结束

bool Vector3D::operator == (const Vector3D& v)
{

    return (abs(m_x - v.m_x) < 1e-8) && (abs(m_y - v.m_y) < 1e-8) && (abs(m_z - v.m_z) < 1e-8);
}

Vector3D Vector3D::operator - () const
{
    return Vector3D(-m_x, -m_y, -m_z);
} //成员函数operator -结束

double Vector3D::mf_getLength()  const
{
    return sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
} //成员函数mf_getLength结束

Vector3D Vector3D::mf_getPerpendicularVector() const
{
    Vector3D vecReturn;
    if (fabs(m_y) < fabs(m_z))
    {
        vecReturn.m_x = m_z;
        vecReturn.m_y = 0.0;
        vecReturn.m_z = -m_x;
    }
    else
    {
        vecReturn.m_x = -m_y;
        vecReturn.m_y = m_x;
        vecReturn.m_z = 0.0;
    }
    return vecReturn;
} //成员函数mf_getPerpendicularVector结束

void Vector3D::mf_normalize()
{
    double a = mf_getLength();
    (*this) /= a; // 注意: 这里没有处理除数为0的情况
} //成员函数mf_normalize结束

void Vector3D::mf_setValue(double newx, double newy, double newz)
{
    m_x = newx;
    m_y = newy;
    m_z = newz;
} //成员函数mf_setValue结束

Face::Face(int m0, int m1, int m2)
{
    v0 = m0;
    v1 = m1;
    v2 = m2;
}

// CP_PointVector implementation
// ////////////////////////////////////////////////////////////////////////////

Point2D operator + (const Point2D& p, const Vector2D& v)
{
    return Point2D(p.m_x + v.m_x, p.m_y + v.m_y);
} //函数operator +结束

Point2D operator - (const Point2D& pt, const Vector2D& v)
{
    return Point2D(pt.m_x - v.m_x, pt.m_y - v.m_y);
} //函数operator -结束

Vector2D operator - (const Point2D& p, const Point2D& q)
{
    return Vector2D(p.m_x - q.m_x, p.m_y - q.m_y);
} //函数operator -结束

Point3D operator + (const Point3D& pt, const Vector3D& v)
{
    return Point3D(pt.m_x + v.m_x, pt.m_y + v.m_y, pt.m_z + v.m_z);
} //函数operator +结束

Point3D operator - (const Point3D& pt, const Vector3D& v)
{
    return Point3D(pt.m_x - v.m_x, pt.m_y - v.m_y, pt.m_z - v.m_z);
} //函数operator -结束

Vector3D operator - (const Point3D& p, const Point3D& q)
{
    return Vector3D(p.m_x - q.m_x, p.m_y - q.m_y, p.m_z - q.m_z);
} //函数operator -结束

Vector2D operator + (const Vector2D& u, const Vector2D& v)
{
    return Vector2D(u.m_x + v.m_x, u.m_y + v.m_y);
} //函数operator +结束

Vector2D operator - (const Vector2D& u, const Vector2D& v)
{
    return Vector2D(u.m_x - v.m_x, u.m_y - v.m_y);
} //函数operator -结束

// 点积
double  operator * (const Vector2D& u, const Vector2D& v)
{
    return u.m_x * v.m_x + u.m_y * v.m_y;
} //函数operator *结束

Vector2D operator * (const Vector2D& v, double num)
{
    return Vector2D(v.m_x * num, v.m_y * num);
} //函数operator *结束

Vector2D operator / (const Vector2D& v, double num)
{
    return Vector2D(v.m_x / num, v.m_y / num); // 注意: 这里没有处理除数为0的情况
} //函数operator /结束

Vector3D operator + (const Vector3D& u, const Vector3D& v)
{
    return Vector3D(u.m_x + v.m_x, u.m_y + v.m_y, u.m_z + v.m_z);
} //函数operator +结束

Vector3D operator - (const Vector3D& u, const Vector3D& v)
{
    return Vector3D(u.m_x - v.m_x, u.m_y - v.m_y, u.m_z - v.m_z);
} //函数operator -结束

// 点积
double operator * (const Vector3D& u, const Vector3D& v)
{
    return (u.m_x * v.m_x + u.m_y * v.m_y + u.m_z * v.m_z);
} //函数operator *结束

// 叉积
Vector3D operator ^ (const Vector3D& u, const Vector3D& v)
{
    return Vector3D((u.m_y * v.m_z) - (u.m_z * v.m_y),
        (-u.m_x * v.m_z) + (u.m_z * v.m_x),
        (u.m_x * v.m_y) - (u.m_y * v.m_x));
} //函数operator ^结束

Vector3D operator * (const Vector3D& v, double num)
{
    return Vector3D(v.m_x * num, v.m_y * num, v.m_z * num);
} //函数operator *结束

Vector3D operator / (const Vector3D& v, double num)
{
    num = 1.0 / num; // 注意: 这里没有处理除数为0的情况
    return Vector3D(v.m_x * num, v.m_y * num, v.m_z * num);
} //函数operator /结束

double  GetDistance(const Point3D p1, const Point3D p2)
{
    Vector3D p1_2 = p1 - p2;
    return p1_2.mf_getLength();
}

Vector3D& NormalizeVector3D(Vector3D& n)
{
    double len = sqrt(n.m_x * n.m_x + n.m_y * n.m_y + n.m_z * n.m_z);
   //// assert(len != 0);
    if (len != 0) {
        std::cout << "ERROR in NormalizeVector3D(Vector3D& n) " << std::endl;
    };

    n.m_x = n.m_x / len;
    n.m_y = n.m_y / len;
    n.m_z = n.m_z / len;
    return n;
}

double SinValue(const Vector3D& a, const Vector3D& b, const Vector3D& c)
{
    double lab = (b - a).mf_getLength();
    double lac = (c - a).mf_getLength();
    return ((b - a) ^ (c - a)).mf_getLength() / (lab * lac);
}

double CosValue(const Vector3D& a, const Vector3D& b, const Vector3D& c)
{
    double lab = (b - a).mf_getLength();
    double lac = (c - a).mf_getLength();
    double lbc = (b - c).mf_getLength();
    double lab2 = lab * lab;
    double lac2 = lac * lac;
    double lbc2 = lbc * lbc;
    return (lab2 + lac2 - lbc2) / (2.0 * lab * lac);
}

double CotValue(const Vector3D& a, const Vector3D& b, const Vector3D& c)
{
    double cosx = CosValue(a, b, c);
    double sinx = MaxValue(SinValue(a, b, c), 1e-8);
    double cotx = cosx / sinx;
    return cotx;
}

double MaxValue(double value1, double value2)
{
    if (value1 > value2)
        return value1;
    else
        return value2;
}










 

class CQuaternion
{
public:
    double w;
    double x;
    double y;
    double z;

public:
    CQuaternion();
    //~CQuaternion();

    CQuaternion(double a, double b, double c, double d);
    CQuaternion  operator +(const CQuaternion&);
    CQuaternion  operator -(const CQuaternion&);
    CQuaternion  operator *(const double&);
    CQuaternion  operator /(const double&);
    CQuaternion  operator *(const CQuaternion&);
    void R2Q(const double&, const double&, const double&, const double&);
    void Q2R(Eigen::Matrix4d& q_r);
    int Sign(double x) { return (x > 0) ? 1 : -1; };
    void GetInverse();
    double GetNorm();
    void Normalize();
    double DotProduct(const CQuaternion& q);
    void RotationMatrix2Qua(const Eigen::Matrix4d& mat);
    CQuaternion& Slerp(CQuaternion& qua1, CQuaternion& qua2, double t);

};

CQuaternion::CQuaternion()
{
    x = 0;
    y = 0;
    z = 0;
    w = 0;
}
CQuaternion::CQuaternion(double a, double b, double c, double d)
{
    x = a;
    y = b;
    z = c;
    w = d;
}
//
CQuaternion CQuaternion::operator +(const CQuaternion& b)
{
    CQuaternion result;
    result.x = x + b.x;
    result.y = y + b.y;
    result.z = z + b.z;
    result.w = w + b.w;
    return result;
}
//
CQuaternion CQuaternion::operator -(const CQuaternion& b)
{
    CQuaternion result;
    result.x = x - b.x;
    result.y = y - b.y;
    result.z = z - b.z;
    result.w = w - b.w;
    return result;
}
//
CQuaternion CQuaternion::operator *(const double& coff)
{
    CQuaternion result;
    result.x = x * coff;
    result.y = y * coff;
    result.z = z * coff;
    result.w = w * coff;
    return result;
}
//
CQuaternion CQuaternion::operator /(const double& coff)
{
    CQuaternion result;
    result.x = x / coff;
    result.y = y / coff;
    result.z = z / coff;
    result.w = w / coff;
    return result;
}
//
CQuaternion CQuaternion::operator *(const CQuaternion& b)
{
    CQuaternion result;
    result.x = w * b.x + x * b.w + y * b.z - z * b.y;
    result.y = w * b.y - x * b.z + y * b.w + z * b.x;
    result.z = w * b.z + x * b.y - y * b.x + z * b.w;
    result.w = w * b.w - x * b.x - y * b.y - z * b.z;
    return result;
}

//
void CQuaternion::R2Q(const double& theta, const double& a, const double& b, const double& c)
{
    double sin_2 = sin(theta * M_PI / 360.0);
    double cos_2 = cos(theta * M_PI / 360.0);
    double len = sqrt(a * a + b * b + c * c);
    x = a / len * sin_2;
    y = b / len * sin_2;
    z = c / len * sin_2;
    w = cos_2;
}
//
void CQuaternion::Q2R(Eigen::Matrix4d & q_r)
{
    q_r(0, 0) = 1 - 2 * y * y - 2 * z * z;
    q_r(0, 1) = 2 * x * y - 2 * z * w;
    q_r(0, 2) = 2 * y * w + 2 * x * z;
    q_r(0, 3) = 0;

    q_r(1, 0) = 2 * x * y + 2 * z * w;
    q_r(1, 1) = 1 - 2 * x * x - 2 * z * z;
    q_r(1, 2) = -2 * x * w + 2 * y * z;
    q_r(1, 3) = 0;

    q_r(2, 0) = -2 * y * w + 2 * x * z;
    q_r(2, 1) = 2 * x * w + 2 * y * z;
    q_r(2, 2) = 1 - 2 * x * x - 2 * y * y;
    q_r(2, 3) = 0;

    q_r(3, 0) = 0;
    q_r(3, 1) = 0;
    q_r(3, 2) = 0;
    q_r(3, 3) = 1;
}

//
double CQuaternion::GetNorm()
{
    double result;
    result = sqrt(x * x + y * y + z * z + w * w);
    return result;
}

//
void CQuaternion::Normalize()
{
    if (!(x || y || z || w))
        return;
    double norm = GetNorm();

    x = x / norm;
    y = y / norm;
    z = z / norm;
    w = w / norm;
}
//
void CQuaternion::GetInverse()
{
    double normex = GetNorm() * GetNorm();
    x = (-x) / normex;
    y = (-y) / normex;
    z = (-z) / normex;
    w = w;
}

//
void CQuaternion::RotationMatrix2Qua(const Eigen::Matrix4d& rotation)
{
    /*
    x = (rotation(0,0) + rotation(1,1) + rotation(2,2) + 1.0) / 4.0;
    y = (rotation(0,0) - rotation(1,1) - rotation(2,2) + 1.0) / 4.0;
    z = (-rotation(0,0) + rotation(1,1) - rotation(2,2) + 1.0) / 4.0;
    w = (-rotation(0,0) - rotation(1,1) + rotation(2,2) + 1.0) / 4.0;
    x = sqrt(max(x, 0.0));
    y = sqrt(max(y, 0.0));
    z = sqrt(max(z, 0.0));
    w = sqrt(max(w, 0.0));

    if (x >= y && x >= z && x >= w)
    {
        x *= 1.0;
        y *= Sign(rotation(2,1) - rotation(1,2));
        z *= Sign(rotation(0,2) - rotation(2,0));
        w *= Sign(rotation(1,0) - rotation(0,1));
    }
    else if (y >= x && y >= z && y >= w)
    {
        x *= Sign(rotation(2,1) - rotation(1,2));
        y *= 1.0;
        z *= Sign(rotation(1,0) + rotation(0,1));
        w *= Sign(rotation(0,2) + rotation(2,0));
    }
    else if (z >= x && z >= y && z >= w)
    {
        x *= Sign(rotation(0,2) - rotation(2,0));
        y *= Sign(rotation(1,0) + rotation(0,1));
        z *= 1.0;
        w *=Sign(rotation(2,1) + rotation(1,2));
    }
    else if (w >= x && w >= y && w >= z)
    {
        x *= Sign(rotation(1,0) - rotation(0,1));
        y *= Sign(rotation(0,2) + rotation(2,0));
        z *= Sign(rotation(2,1) + rotation(1,2));
        w *= 1.0;
    }

    Normalize();
    */


    double e_1_2 = rotation(1, 2);
    double e_2_0 = rotation(2, 0);
    double angle = 0;
    if (e_1_2 != 0)
    {
        angle = asin(-e_1_2) / M_PI * 180;
        R2Q(angle, 1, 0, 0);
    }
    else if (e_2_0 != 0)
    {
        angle = asin(-e_2_0) / M_PI * 180;
        R2Q(angle, 0, 1, 0);
    }
}

//
CQuaternion& CQuaternion::Slerp(CQuaternion& qua1, CQuaternion& qua2, double t)
{
    qua1.Normalize();
    qua2.Normalize();

    float a0, a1;

    double cosValue = qua1.DotProduct(qua2);
    if (cosValue < 0)
    {
        /*
        w = -w;
        x = -x;
        y = -y;
        z = -z;
        */

        qua2.w = -qua2.w;
        qua2.x = -qua2.x;
        qua2.y = -qua2.y;
        qua2.z = -qua2.z;

        cosValue = -cosValue;
    }

    if (cosValue > 0.999999)
    {
        a0 = t;
        a1 = 1 - t;
    }

    else
    {
        double sinValue = sqrt(1 - cosValue * cosValue);
        double ang = asin(sinValue);
        double oneOverSin = 1.0 / sinValue;
        a0 = sin((1.0 - t) * ang) * oneOverSin;
        a1 = sin(t * ang) * oneOverSin;
    }

    w = a0 * qua1.w + a1 * qua2.w;
    x = a0 * qua1.x + a1 * qua2.x;
    y = a0 * qua1.y + a1 * qua2.y;
    z = a0 * qua1.z + a1 * qua2.z;

    return (*this);
}

//
double CQuaternion::DotProduct(const CQuaternion& q)
{
    double result = 0;
    result += (x * q.x);
    result += (y * q.y);
    result += (z * q.z);
    result += (w * q.w);

    return result;
}




/////
class SimpleCompute
{
public:
    SimpleCompute(void);
    ~SimpleCompute(void);

    static Point3D ComputeMatrixMultiPoint(Eigen::Matrix4d Mt, Point3D point);
    static Point3D ComputeMatrixMultiPoint(Eigen::Matrix3d Mt, Point3D point);
    static Vector3D ComputeMatrixMutiVector(Eigen::Matrix3d Mt, Vector3D vec);
    static double GetTriangleArea(const Point3D& v1, const Point3D& v2, const Point3D& v3);
    static Vector3D GetTriangleVertexGradient(Vector3D a, Vector3D b);
    static Eigen::Matrix4d Rotation2Matrix(double angle, int x, int y, int z);
    static Eigen::Matrix4d Rotation2Matrix(double angle_x, double angle_y, double angle_z, bool x, bool y, bool z);
    static Eigen::Matrix4d Translate2Matrix(double x, double y, double z);
    static Eigen::Matrix4d Scale2Matrix(double x, double y, double z);
    static Eigen::Matrix4d MatrixMultNum(const Eigen::Matrix4d mat, double factor);
    static Vector3D getNormalOfTri(const Point3D p1, const Point3D p2, const Point3D p3);
    static double ComputeMeshVolume(Polyhedron& desMesh);
    static void  EvaluateVNormalByArea(Polyhedron& desMesh, std::vector<Vector3D>& vNormal);
};



SimpleCompute::SimpleCompute(void)
{
}


SimpleCompute::~SimpleCompute(void)
{
}

Point3D SimpleCompute::ComputeMatrixMultiPoint(Eigen::Matrix4d Mt, Point3D point)
{
    /*
    |m1  m2  m3  m4 |   |x|
    |m5  m6  m7  m8 | * |y|
    |m9  m10 m11 m12|	|z|
    |m13 m14 m15 m16|	|1|
    */

    Point3D tar;
    tar.m_x = Mt(0, 0) * point.m_x + Mt(0, 1) * point.m_y + Mt(0, 2) * point.m_z + Mt(0, 3);
    tar.m_y = Mt(1, 0) * point.m_x + Mt(1, 1) * point.m_y + Mt(1, 2) * point.m_z + Mt(1, 3);
    tar.m_z = Mt(2, 0) * point.m_x + Mt(2, 1) * point.m_y + Mt(2, 2) * point.m_z + Mt(2, 3);

    return tar;
}

Point3D SimpleCompute::ComputeMatrixMultiPoint(Eigen::Matrix3d Mt, Point3D point)
{
    /*
    |m1  m2  m3 |   |x|
    |m4  m5  m6 | * |y|
    |m7  m8  m9 |	|z|
    */

    Point3D tar;
    tar.m_x = Mt(0, 0) * point.m_x + Mt(0, 1) * point.m_y + Mt(0, 2) * point.m_z;
    tar.m_y = Mt(1, 0) * point.m_x + Mt(1, 1) * point.m_y + Mt(1, 2) * point.m_z;
    tar.m_z = Mt(2, 0) * point.m_x + Mt(2, 1) * point.m_y + Mt(2, 2) * point.m_z;

    return tar;
}

Vector3D SimpleCompute::ComputeMatrixMutiVector(Eigen::Matrix3d Mt, Vector3D vec)
{
    Vector3D tar;
    tar.m_x = Mt(0, 0) * vec.m_x + Mt(0, 1) * vec.m_y + Mt(0, 2) * vec.m_z;
    tar.m_y = Mt(1, 0) * vec.m_x + Mt(1, 1) * vec.m_y + Mt(1, 2) * vec.m_z;
    tar.m_z = Mt(2, 0) * vec.m_x + Mt(2, 1) * vec.m_y + Mt(2, 2) * vec.m_z;

    return tar;
}

double SimpleCompute::GetTriangleArea(const Point3D& v1, const Point3D& v2, const Point3D& v3)
{
    Vector3D a = v2 - v1;
    Vector3D b = v3 - v1;
    Vector3D c = v2 - v3;

    double aa = a.mf_getLength();
    double bb = b.mf_getLength();
    double cc = c.mf_getLength();
    double p = (aa + bb + cc) / 2.0;
    double area = sqrt(p * (p - aa) * (p - bb) * (p - cc));

    return area;
}

Vector3D SimpleCompute::GetTriangleVertexGradient(Vector3D a, Vector3D b)
{
    Vector3D high, c;
    double len2 = 0;
    a = a * (-1);
    b = b * (-1);
    c = a - b;

    double dotres = a * c;

    double lenc2 = c.m_x * c.m_x + c.m_y * c.m_y + c.m_z * c.m_z;
    double ratio = dotres / (lenc2);

    high = (b - a) * ratio + a;
    high = high * (-1);

    len2 = high.m_x * high.m_x + high.m_y * high.m_y + high.m_z * high.m_z;
    high.m_x /= len2;
    high.m_y /= len2;
    high.m_z /= len2;

    return high;
}

Eigen::Matrix4d SimpleCompute::Rotation2Matrix(double angle_x, double angle_y, double angle_z, bool x, bool y, bool z)
{
    Eigen::Matrix4d rotation_x;
    Eigen::Matrix4d rotation_y;
    Eigen::Matrix4d rotation_z;
    Eigen::Matrix4d result;

    rotation_y = Rotation2Matrix(angle_x, 0, 1, 0);
    rotation_x = Rotation2Matrix(angle_y, 1, 0, 0);

    result = rotation_y * rotation_x;
    return result;
}

Eigen::Matrix4d SimpleCompute::Rotation2Matrix(double angle, int x, int y, int z)
{
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();

    double hudu = angle / 180.0 * PI;
    double cosa = cos(hudu);
    double sina = sin(hudu);

    if (1 == x)
    {
        result(1, 1) = cosa;
        result(1, 2) = sina;
        result(2, 1) = -sina;
        result(2, 2) = cosa;
    }
    else if (1 == y)
    {
        result(0, 0) = cosa;
        result(0, 2) = -sina;
        result(2, 0) = sina;
        result(2, 2) = cosa;
    }
    else if (1 == z)
    {
        result(0, 0) = cosa;
        result(0, 1) = sina;
        result(1, 0) = -sina;
        result(1, 1) = cosa;
    }
    return result;
}

Eigen::Matrix4d SimpleCompute::Translate2Matrix(double x, double y, double z)
{
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();

    result(0, 3) = x;
    result(1, 3) = y;
    result(2, 3) = z;

    return result;
}

Eigen::Matrix4d SimpleCompute::Scale2Matrix(double x, double y, double z)
{
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();

    result(0, 0) = x;
    result(1, 1) = y;
    result(2, 2) = z;

    return result;
}

Eigen::Matrix4d SimpleCompute::MatrixMultNum(const Eigen::Matrix4d mat, double factor)
{
    Eigen::Matrix4d mt = mat * factor;
    return mt;
}

Vector3D SimpleCompute::getNormalOfTri(const Point3D p1, const Point3D p2, const Point3D p3)
{
    Vector3D v1(p2.m_x - p1.m_x, p2.m_y - p1.m_y, p2.m_z - p1.m_z);
    Vector3D v2(p3.m_x - p1.m_x, p3.m_y - p1.m_y, p3.m_z - p1.m_z);

    Vector3D result = (v1 ^ v2);
    result.mf_normalize();

    return result;
}

//double SimpleCompute::ComputeMeshVolume(Polyhedron& desMesh)
//{
//    double  minZ = INFINITY;
//    for (TriMesh::VertexIter vi = desMesh.vertices_begin(); vi != desMesh.vertices_end(); vi++)
//    {
//        if (desMesh.point(vi)[2] < minZ)
//            minZ = desMesh.point(vi)[2];
//    }
//    minZ -= 1;
//
//
//    Vector3D upVector(0, 0, 1);
//    double meshVolume = 0;
//
//    
//    desMesh.request_face_normals();
//    desMesh.request_vertex_normals();
//    desMesh.update_normals();
//
//    for (TriMesh::FaceIter f_it = desMesh.faces_begin(); f_it != desMesh.faces_end(); f_it++)
//    {
//        TriMesh::Normal fn = desMesh.normal(f_it);
//        Vector3D f_normal = Vector3D(fn[0], fn[1], fn[2]);
//        double dianji = f_normal * upVector;
//        int signx = dianji >= 0 ? 1 : -1;
//        double prismHeight = 0;
//
//        int i = 0;
//        Point3D triVertex[3];
//        for (TriMesh::FaceVertexIter fv_it = desMesh.fv_begin(f_it); fv_it != desMesh.fv_end(f_it); fv_it++)
//        {
//            TriMesh::Point point = desMesh.point(fv_it);
//            triVertex[i++] = Point3D(point[0], point[1], point[2]);
//            prismHeight += (point[2] - minZ);
//        }
//        prismHeight /= 3.0;
//
//
//        Vector2D edge_xy_1(triVertex[1].m_x - triVertex[0].m_x, triVertex[1].m_y - triVertex[0].m_y);
//        Vector2D edge_xy_2(triVertex[2].m_x - triVertex[1].m_x, triVertex[2].m_y - triVertex[1].m_y);
//        double area_xy = edge_xy_1 ^ edge_xy_2 * signx;
//        double prismVolume = 1 / 3.0 * area_xy * prismHeight;
//        meshVolume += prismVolume;
//    }
//
//    return meshVolume;
//}

double SimpleCompute::ComputeMeshVolume(Polyhedron& desMesh)
{
    double minZ = std::numeric_limits<double>::infinity();

    // Find the lowest z-coordinate
    for (auto v = desMesh.vertices_begin(); v != desMesh.vertices_end(); ++v) {
        if (v->point().z() < minZ)
            minZ = v->point().z();
    }
    minZ -= 1.0;

    Vector3D upVector(0, 0, 1);
    double meshVolume = 0.0;

    for (auto f = desMesh.facets_begin(); f != desMesh.facets_end(); ++f) {
        HFCC h = f->facet_begin();
        std::vector<Point3D> triVertex;

        do {
            const auto& p = h->vertex()->point();
            triVertex.push_back(Point3D(p.x(), p.y(), p.z()));
        } while (++h != f->facet_begin());

        if (triVertex.size() != 3) continue;  // Skip non-triangle faces

        // Compute face normal
        Vector3D e1 = triVertex[1] - triVertex[0];
        Vector3D e2 = triVertex[2] - triVertex[1];
        Vector3D f_normal = e1 ^ e2;
        f_normal.mf_normalize();

        double dot = f_normal * upVector;
        int signx = (dot >= 0.0) ? 1 : -1;

        // Average height of triangle above base
        double prismHeight = (triVertex[0].m_z + triVertex[1].m_z + triVertex[2].m_z) / 3.0 - minZ;

        // Area of projection on XY-plane
        Vector2D edge1(triVertex[1].m_x - triVertex[0].m_x, triVertex[1].m_y - triVertex[0].m_y);
        Vector2D edge2(triVertex[2].m_x - triVertex[1].m_x, triVertex[2].m_y - triVertex[1].m_y);
        double area_xy = (edge1 ^ edge2) * signx;

        // Prism volume
        double prismVolume = (1.0 / 3.0) * area_xy * prismHeight;
        meshVolume += prismVolume;
    }

    return meshVolume;
}



//void SimpleCompute::EvaluateVNormalByArea(TriMesh& desMesh, std::vector<Vector3D>& vNormal)
//{
//    desMesh.request_face_normals();
//    desMesh.request_vertex_normals();
//    desMesh.update_normals();
//
//    vNormal.resize(desMesh.n_vertices());
//
//    for (TriMesh::VertexIter vi = desMesh.vertices_begin(); vi != desMesh.vertices_end(); vi++)
//    {
//        double sumArea = 0;
//        vector<double> triAreas;
//        vector<Vector3D> triFaceNorm;
//
//
//        for (TriMesh::VertexFaceIter vf_i = desMesh.vf_begin(vi); vf_i != desMesh.vf_end(vi); vf_i++)
//        {
//            int i = 0;
//            Point3D triPoint[3];
//            triFaceNorm.push_back(Vector3D(desMesh.normal(vf_i)[0], desMesh.normal(vf_i)[1], desMesh.normal(vf_i)[2]));
//
//            for (TriMesh::FaceVertexIter fv_i = desMesh.fv_begin(vf_i); fv_i != desMesh.fv_end(vf_i); fv_i++)
//            {
//                TriMesh::Point point = desMesh.point(fv_i);
//                triPoint[i++] = Point3D(point[0], point[1], point[2]);
//            }
//
//            double area = SimpleCompute::GetTriangleArea(triPoint[0], triPoint[1], triPoint[2]);
//            triAreas.push_back(area);
//            sumArea += area;
//        }
//
//        Vector3D avgNorm;
//        for (int i = 0; i < triAreas.size(); i++)
//        {
//            double rato = triAreas[i] / sumArea;
//            avgNorm += triFaceNorm[i] * rato;
//        }
//
//        avgNorm.mf_normalize();
//        vNormal[vi.handle().idx()] = avgNorm;
//
//
//        // Vector3D vNorm(desMesh.normal(vi)[0],desMesh.normal(vi)[1],desMesh.normal(vi)[2]);
//        // vNormal[vi.handle().idx()] = vNorm;
//    }
//}

void SimpleCompute::EvaluateVNormalByArea(Polyhedron& desMesh, std::vector<Vector3D>& vNormal)
{
    vNormal.resize(desMesh.size_of_vertices(), Vector3D(0, 0, 0));

    for (auto v = desMesh.vertices_begin(); v != desMesh.vertices_end(); ++v)
    {
        double sumArea = 0.0;
        std::vector<double> triAreas;
        std::vector<Vector3D> triFaceNorm;

        //auto hc = v->vertex_begin();
        HFCC h = v->vertex_begin()->facet_begin();
        HFCC h_orig = h;

        CGAL_assertion(h != nullptr);
        do {
            if (!h->is_border()) {
                // Get triangle points
                Point3D triPoint[3];
                triPoint[0] = Point3D(h->vertex()->point().x(), h->vertex()->point().y(), h->vertex()->point().z());
                triPoint[1] = Point3D(h->next()->vertex()->point().x(), h->next()->vertex()->point().y(), h->next()->vertex()->point().z());
                triPoint[2] = Point3D(h->next()->next()->vertex()->point().x(), h->next()->next()->vertex()->point().y(), h->next()->next()->vertex()->point().z());

                // Compute face normal
                Vector3D e1 = triPoint[1] - triPoint[0];
                Vector3D e2 = triPoint[2] - triPoint[0];
                Vector3D normal = e1 ^ e2;
                normal.mf_normalize();
                triFaceNorm.push_back(normal);

                double area = SimpleCompute::GetTriangleArea(triPoint[0], triPoint[1], triPoint[2]);
                triAreas.push_back(area);
                sumArea += area;
            }
            h = h->vertex()->vertex_begin()->facet_begin();
        } while (++h != h_orig);

        Vector3D avgNorm(0, 0, 0);
        for (size_t i = 0; i < triAreas.size(); ++i) {
            double ratio = (sumArea > 0) ? (triAreas[i] / sumArea) : 0.0;
            avgNorm += triFaceNorm[i] * ratio;
        }
        avgNorm.mf_normalize();

        vNormal[v->id()] = avgNorm;
    }
}



class MeshLaplaceSolver
{
public:
    MeshLaplaceSolver(void);
    MeshLaplaceSolver(Polyhedron mesh);
    MeshLaplaceSolver(Polyhedron mesh, const std::vector<bool> isctrlv);
    ~MeshLaplaceSolver(void);

public:
    Polyhedron m_desMesh;
   // OpenMesh::VPropHandleT<std::vector<double>> vertexLPLWeight;

   // CGAL::property_map_selector<Polyhedron, std::vector<double>>vertexLPLWeight;

    std::unordered_map<Polyhedron::Vertex_const_handle, std::vector<double>> vertexLPLWeightMap;
     boost::associative_property_map< std::unordered_map<Polyhedron::Vertex_const_handle, std::vector<double>>> vertexLPLWeight = boost::make_assoc_property_map(vertexLPLWeightMap);


    Eigen::SparseMatrix<double> A;

private:
    std::vector<bool> m_isControlVertex;
    Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int> > solver;

    Eigen::VectorXd B, X;

public:
    void SetControlVertex(const std::vector<bool> isctrlv);
    void SetDesMesh(Polyhedron& mesh_) { m_desMesh = mesh_; };
    Eigen::VectorXd& LplacianSolve();
    void ComputeVertexLaplacianWeight();
    void ComputeLalacianMatrixA();
    void SetRightHandB(Eigen::VectorXd& rightHand) { B = rightHand; }
};




MeshLaplaceSolver::MeshLaplaceSolver(void)
{
}

MeshLaplaceSolver::MeshLaplaceSolver(Polyhedron mesh)
{
    m_desMesh = mesh;
  //  CGAL::set_halfedge_and_vertex_id(m_desMesh);
}

MeshLaplaceSolver::MeshLaplaceSolver(Polyhedron mesh, const std::vector<bool> isctrlv)
{
    m_desMesh = mesh;
    m_isControlVertex = isctrlv;
}

MeshLaplaceSolver::~MeshLaplaceSolver(void)
{
}

void MeshLaplaceSolver::SetControlVertex(const std::vector<bool> isctrlv)
{
    m_isControlVertex.clear();
    m_isControlVertex = isctrlv;
}


//void MeshLaplaceSolver::ComputeVertexLaplacianWeight()
//{
//    m_desMesh.add_property(vertexLPLWeight);
//    for (TriMesh::VertexIter v_it = m_desMesh.vertices_begin(); v_it != m_desMesh.vertices_end(); ++v_it)
//    {
//        Vector3D currentPosition = Vector3D(m_desMesh.point(v_it)[0], m_desMesh.point(v_it)[1], m_desMesh.point(v_it)[2]);
//
//        std::vector<Vector3D> neighborPoints;
//        for (TriMesh::VertexVertexIter vv_it = m_desMesh.vv_iter(v_it); vv_it; ++vv_it)
//            neighborPoints.push_back(Vector3D(m_desMesh.point(vv_it)[0], m_desMesh.point(vv_it)[1], m_desMesh.point(vv_it)[2]));
//
//        bool is_boder_vertex = m_desMesh.is_boundary(v_it);
//        int numNeighbors = (int)neighborPoints.size();
//        std::vector<double> weights(numNeighbors, 0);
//
//        for (int i = 0; i < numNeighbors; i++)
//        {
//            double w1 = CotValue(neighborPoints[(i + numNeighbors - 1) % numNeighbors], currentPosition, neighborPoints[i]);
//            double w2 = CotValue(neighborPoints[(i + 1) % numNeighbors], currentPosition, neighborPoints[i]);
//
//            if (is_boder_vertex)
//            {
//                if (i == 0)
//                    w1 = CotValue(neighborPoints[i], currentPosition, neighborPoints[i]);
//                if (i == numNeighbors - 1)
//                    w2 = CotValue(neighborPoints[i], currentPosition, neighborPoints[i]);
//            }
//            weights[i] = 0.5 * (w1 + w2);
//            weights[i] = MaxValue(weights[i], EPSILON);
//        }
//        m_desMesh.property(vertexLPLWeight, v_it) = weights;
//    }
//}


void MeshLaplaceSolver::ComputeVertexLaplacianWeight()
{
    for (auto v = m_desMesh.vertices_begin(); v != m_desMesh.vertices_end(); ++v)
    {
        const Point& p = v->point();
        Vector3D currentPosition(p.x(), p.y(), p.z());

        std::vector<Vector3D> neighborPoints;
        std::vector<Polyhedron::Vertex_const_handle> neighbors;

        Polyhedron::Halfedge_around_vertex_const_circulator h = v->vertex_begin(), done(h);
        bool is_border = false;

        CGAL_assertion(h != NULL);
        do {
            if (h->is_border())
                is_border = true;

            auto neighbor = h->opposite()->vertex();
            neighbors.push_back(neighbor);

            const Point& np = neighbor->point();
            neighborPoints.emplace_back(np.x(), np.y(), np.z());

        } while (++h != done);

        int numNeighbors = static_cast<int>(neighborPoints.size());
        std::vector<double> weights(numNeighbors, 0.0);

        for (int i = 0; i < numNeighbors; i++)
        {
            const Vector3D& prev = neighborPoints[(i + numNeighbors - 1) % numNeighbors];
            const Vector3D& next = neighborPoints[(i + 1) % numNeighbors];
            const Vector3D& curr = neighborPoints[i];

            double w1 = CotValue(prev, currentPosition, curr);
            double w2 = CotValue(next, currentPosition, curr);

            if (is_border)
            {
                if (i == 0)
                    w1 = CotValue(curr, currentPosition, curr);
                if (i == numNeighbors - 1)
                    w2 = CotValue(curr, currentPosition, curr);
            }

            weights[i] = 0.5 * (w1 + w2);
            weights[i] = MaxValue(weights[i], EPSILON); // Prevent zero weights
        }
   
        vertexLPLWeight[v] = weights;
    }
}

void MeshLaplaceSolver::ComputeLalacianMatrixA()
{
    ComputeVertexLaplacianWeight();

    assert(!m_isControlVertex.empty());

    int numVertices = m_desMesh.size_of_vertices();
    A.resize(numVertices, numVertices);

    for (auto v = m_desMesh.vertices_begin(); v != m_desMesh.vertices_end(); ++v)
    {
        int vindex = v->id();

        if (m_isControlVertex[vindex]) // control points
        {
            A.coeffRef(vindex, vindex) = 1.0;
        }
        else
        {
            double sum = 0.0;

            // Get Laplacian weights for vertex v
            const std::vector<double>& weights = vertexLPLWeight[v];
            int id = 0;

            // Circulate over neighboring vertices
            Polyhedron::Halfedge_around_vertex_const_circulator h = v->vertex_begin();
            Polyhedron::Halfedge_around_vertex_const_circulator h_end = h;

            do {
                int vvindex = h->opposite()->vertex()->id();

                A.coeffRef(vindex, vvindex) = -weights[id];
                sum += weights[id];
                ++id;
                ++h;
            } while (h != h_end);

            A.coeffRef(vindex, vindex) = sum;
        }
    }


    //pre QR decompose
    A.makeCompressed();
    solver.compute(A);
}


Eigen::VectorXd& MeshLaplaceSolver::LplacianSolve()
{
    X = solver.solve(B);
    return X;
}








class ControlVertex
{
public:
    ControlVertex(void) {};
    ~ControlVertex(void) {};

public:
    Polyhedron m_mesh;
    std::vector<double>  m_freeVertexWeight;
    std::vector<Point3D> m_posOfHand;
    std::vector<int>     m_HandVindex;
    std::vector<int>     m_fixedVindex;

public:
    void LoadHandVertex();
    std::vector<bool>  GetVertexProperty();
    void   Draw();
    void   HandTransform(Eigen::Matrix4d mt);
    void   ComputeFreeVertexWeight();
    void   UpdateHandPos();
    void   Clear();
    void   Reset();
};


void ControlVertex::LoadHandVertex()
{
    m_HandVindex.clear();
    int vindex = 0;

    //CFileDialog file(true);
    //file.m_ofn.lpstrFilter = _T("txt file(*.txt)\0*.txt\0*.*\0\0");
    //file.m_ofn.lpstrTitle = _T("OPEN");
    //if (file.DoModal() == IDOK)
    //{
    //    CString m_filePath = file.GetPathName();
    //    ifstream  read_(m_filePath);
    //    while (!read_.eof())
    //    {
    //        read_ >> vindex;
    //        m_HandVindex.push_back(vindex);
    //    }
    //}


    for (auto i : handle_indices) {
        m_HandVindex.push_back(i);
    }


    m_posOfHand.clear();
    for (auto v = m_mesh.vertices_begin(); v != m_mesh.vertices_end(); ++v) {
        m_posOfHand.push_back(Point3D(v->point().x(), v->point().y(), v->point().z()));
    }
    //for (int i = 0; i < m_HandVindex.size(); i++)
    //{
    //  



    //  //  TriMesh::VertexHandle vh = m_mesh->vertex_handle(m_HandVindex[i]);
    //  //  Point3D point(m_mesh->point(vh)[0], m_mesh->point(vh)[1], m_mesh->point(vh)[2]);
    //  //  m_posOfHand.push_back(point);
    //}
}


void ControlVertex::Draw()
{
    if (!m_mesh.empty())
        return;

    glDisable(GL_LIGHTING);

    //draw hand points
    glColor4f(1, 0, 0, 1);
    glPointSize(6);
    glBegin(GL_POINTS);
    for (int i = 0; i < m_posOfHand.size(); i++)
        glVertex3d(m_posOfHand[i].m_x, m_posOfHand[i].m_y, m_posOfHand[i].m_z);
    glEnd();
    glPointSize(1);


    //draw fixed vertex
    glColor4f(1, 1, 1, 1);
    glPointSize(6);
    glBegin(GL_POINTS);


    //for (int i = 0; i < m_fixedVindex.size(); i++)
    //{
    //    TriMesh::Point point = m_mesh->point(TriMesh::VertexHandle(m_fixedVindex[i]));
    //    glVertex3d(point[0], point[1], point[2]);
    //}

    for (int i = 0; i < m_fixedVindex.size(); ++i) {
        int id = m_fixedVindex[i];
        for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
            if (v->id() == id) {
                const auto& p = v->point();
                glVertex3d(p.x(), p.y(), p.z());
                break; // ID found, no need to continue loop
            }
        }
    }


    glEnd();
    glPointSize(1);

    glEnable(GL_LIGHTING);
}



void ControlVertex::HandTransform(Eigen::Matrix4d mt)
{
  //  Point3D center = afxGetMainMesh()->m_centerOfAABB;
  

    Point3D center;
    glm::vec3 minPt(FLT_MAX), maxPt(-FLT_MAX);

    for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
        
            const auto& pt = v->point();
            glm::vec3 p(pt.x(), pt.y(), pt.z());
            minPt = glm::min(minPt, p);
            maxPt = glm::max(maxPt, p);
        
    }

    center.m_x = (minPt.x + maxPt.x) / 2;
    center.m_y = (minPt.y + maxPt.y) / 2;
    center.m_z = (minPt.z + maxPt.z) / 2;


    Eigen::Matrix4d mt1 = SimpleCompute::Translate2Matrix(-center.m_x, -center.m_y, -center.m_z);
    Eigen::Matrix4d mt2 = SimpleCompute::Translate2Matrix(center.m_x, center.m_y, center.m_z);

    Eigen::Matrix4d mat = mt2 * mt * mt1;

    for (int i = 0; i < m_HandVindex.size(); i++)
    {
       m_posOfHand[i] = SimpleCompute::ComputeMatrixMultiPoint(mat, Point3D(m_posOfHand[i].m_x, m_posOfHand[i].m_y, m_posOfHand[i].m_z));
    }
}


std::vector<bool> ControlVertex::GetVertexProperty()
{
    std::vector<bool> isControlVertex(m_mesh.size_of_vertices(), false);
    for (int i = 0; i < m_HandVindex.size(); i++)
        isControlVertex[m_HandVindex[i]] = true;

    for (int j = 0; j < m_fixedVindex.size(); j++)
        isControlVertex[m_fixedVindex[j]] = true;

    return isControlVertex;
}

void ControlVertex::Clear()
{
    m_HandVindex.clear();
    m_fixedVindex.clear();
    m_posOfHand.clear();
    m_freeVertexWeight.clear();
}


void ControlVertex::Reset()
{
    m_posOfHand.clear();

    for (int i = 0; i < m_HandVindex.size(); ++i)
    {
        int vid = m_HandVindex[i];

        for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v)
        {
            if (v->id() == vid)
            {
                const auto& p = v->point();
                Point3D point(p.x(), p.y(), p.z());
                m_posOfHand.push_back(point);
                break;
            }
        }
    }

}





void ControlVertex::ComputeFreeVertexWeight()
{
    std::vector<bool> isControlVertex = GetVertexProperty();

    MeshLaplaceSolver LPLslover(m_mesh);
    LPLslover.SetControlVertex(isControlVertex);
    LPLslover.ComputeLalacianMatrixA();

    int nvertex = m_mesh.size_of_vertices();
    Eigen::VectorXd b(nvertex); b.setZero();

    for (int i = 0; i < m_HandVindex.size(); i++)
    {
        int handIndex = m_HandVindex[i];
        b[handIndex] = 1;
    }
    LPLslover.SetRightHandB(b);

    Eigen::VectorXd  x = LPLslover.LplacianSolve();
    for (int i = 0; i < nvertex; i++)
        m_freeVertexWeight.push_back(1 - x[i]);
}





class Deformation
{
public:
    Polyhedron m_mesh;

public:
    Deformation(void);
    virtual ~Deformation(void);

public:
    virtual void MyDeformation() = 0;
    virtual void Draw() = 0;
    virtual void reset() = 0;
    virtual void InteracTransform(const Eigen::Matrix4d mat, bool localTransform = true) = 0;
};



class PoissonDeformation :public Deformation
{
private:
    Polyhedron  m_static_mesh;
    Eigen::VectorXd divMatri_x;
    Eigen::VectorXd divMatri_y;
    Eigen::VectorXd divMatri_z;
    MeshLaplaceSolver myLPLslover;
    CQuaternion m_quater_fixed;
    CQuaternion m_quater_hand;
    Eigen::Matrix4d m_handTransMat;

public:
    ControlVertex poissonCtrl;

public:
    PoissonDeformation(Polyhedron mesh);
    ~PoissonDeformation(void) {};

    //基类接口
public:
    void MyDeformation();
    void Draw() { poissonCtrl.Draw(); }
    void reset();
    void InteracTransform(const Eigen::Matrix4d mat, bool localTransform = true);

    //核心算法
public:
    void ComputeCoefficientMatrix();
    void ComputeDivergence();
    Vector3D ComputeTriangleDiv(const Point3D& source, const Point3D& vleft, const Point3D& vright, int l, int r);
    void TriangleLocalTransform(Polyhedron::Vertex_handle vh_s, Polyhedron::Vertex_handle vh_l, Polyhedron::Vertex_handle vh_r,
        Point3D& source, Point3D& left, Point3D& right);
    void DeformReset();
};



PoissonDeformation::PoissonDeformation(Polyhedron mesh)
{
    m_mesh = mesh;
    m_static_mesh = mesh;
    m_handTransMat = Eigen::Matrix4d::Identity();
    poissonCtrl.m_mesh = mesh;
    m_quater_fixed.R2Q(0, 1, 0, 0);
}

void PoissonDeformation::reset()
{
    poissonCtrl.Clear();
    m_mesh = m_static_mesh;
    m_handTransMat = Eigen::Matrix4d::Identity();
}


void PoissonDeformation::InteracTransform(const Eigen::Matrix4d mat, bool localTransform)
{
    m_handTransMat = mat * m_handTransMat;
    poissonCtrl.HandTransform(mat);
    m_quater_hand.RotationMatrix2Qua(m_handTransMat);
}











//////////////////////////////////////////////////////////////////////////////////Utility functions/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//glm::mat4 getProjectionMatrix(int width, int height) {
//    float aspect = (float)width / height;
//    if (cameraMode == PERSPECTIVE) {
//        if (height == 0) height = 1;
//
//        return glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);
//    }
//    else {
//        float orthoScale = 2.0f;
//        return glm::ortho(-orthoScale * aspect, orthoScale * aspect, -orthoScale, orthoScale, 0.1f, 100.0f);
//    }
//}
//
//glm::mat4 getViewMatrix() {
//    if (controlMode == ORBIT) {
//        float yawRad = glm::radians(cameraYaw);
//        float pitchRad = glm::radians(cameraPitch);
//        glm::vec3 direction = {
//            cos(yawRad) * cos(pitchRad),
//            sin(pitchRad),
//            sin(yawRad) * cos(pitchRad)
//        };
//        glm::vec3 cameraPos = cameraTarget - direction * cameraDistance;
//        return glm::lookAt(cameraPos, cameraTarget, glm::vec3(0, 1, 0));
//    }
//    else if (controlMode == FREE) {
//        return glm::lookAt(glm::vec3(0, 0, cameraDistance), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
//    }
//    else if (controlMode == MODEL_VIEW) {
//        // Fixed camera; move model instead
//        return glm::lookAt(glm::vec3(0, 0, cameraDistance), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
//    }
//    return glm::mat4(1.0f);
//}

bool pointInPolygon(const glm::vec2& pt, const std::vector<glm::vec2>& poly) {
    int count = 0;
    for (size_t i = 0; i < poly.size(); ++i) {
        glm::vec2 a = poly[i];
        glm::vec2 b = poly[(i + 1) % poly.size()];
        if (((a.y > pt.y) != (b.y > pt.y)) &&
            (pt.x < (b.x - a.x) * (pt.y - a.y) / ((b.y - a.y) + 1e-5f) + a.x)) {
            count++;
        }
    }
    return (count % 2) == 1;
}

std::set<int> getDeformRegion(int totalVertices) {// Compute region as all vertices not in handle or fixed
    std::set<int> region;
    for (int i = 0; i < totalVertices; ++i) {
        if (handle_indices.count(i) == 0 && fixed_indices.count(i) == 0)
            region.insert(i);
    }
    return region;
}

void loadSTLMesh(const std::string& path) {
    if (!mesh.empty()) {
        
        deform->reset();
        mesh.clear();
        handle_indices.clear();
        fixed_indices.clear();
    }

    std::ifstream input(path, std::ios::binary);
    if (!input || !CGAL::IO::read_STL(input, mesh)) {
        std::cerr << "Error: Cannot read STL file: " << path << std::endl;

    }
    input.close();
   
    for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
        original_positions[v->id()] = v->point();
    }
    
    //deformation arap
    // Initialize IDs
    CGAL::set_halfedgeds_items_id(mesh);
    deform = std::make_unique<Surface_mesh_deformation>(mesh);
    deform->set_sre_arap_alpha(10);
   
    
    std::cout << "Loaded mesh: " << mesh.size_of_vertices() << " vertices, " << mesh.size_of_facets() << " triangles\n";

}
//////////////////////////////////////////////////////////////////////////////////Utility functions/////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////forward decl functions//////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//model view
void differentViewsModes(RenderMode mode);
void drawSelectionPloygon();
void drawSelectedRegion();
void drawMouseControlledDeform();
void drawModel(RenderMode mode);

//mouse movement utility
glm::mat4 getProjectionMatrix(GLFWwindow* window);
glm::mat4 getViewMatrix();
glm::vec2 worldToScreen(const glm::vec3& worldPos, const glm::mat4& projection, const glm::mat4& view, int width, int height);

//mouse movement
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void findDraggedVertex(Vertex_iterator& dragged_v);
void mouse_drag_handle(GLFWwindow* window, const glm::mat4& view, const glm::mat4& proj, int width, int height);


//rendering main loop
int framebufferWidth = 1280;
int framebufferHeight = 720;
void updateCameraMovement();
void prepareFramebuffer(glm::mat4& view, glm::mat4& projection);
void renderMainMenu();
void renderSTLViewerWindow();
void renderCameraControlsWindow();
void renderSelectionControls();
void renderMainLoop();

//mainwindow
int mainwindow();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int mainwindow() {
    if (!glfwInit()) return -1;
    window = glfwCreateWindow(1280, 960, "STL Viewer", NULL, NULL);
    if (!window) return -1;
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat light_pos[] = { 0.0f, 5.0f, 5.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

    //mouse movement 
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, key_callback);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    renderMode = SHADED;

    renderMainLoop();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
//////////////////////////////////////////////////////////////////////////////////Model rendering/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void differentViewsModes(RenderMode mode) {
    glPushMatrix();
    if (controlMode == MODEL_VIEW) {
        glScalef(modelScale, modelScale, modelScale);
        glTranslatef(modelOffset.x, modelOffset.y, modelOffset.z);
        glRotatef(modelRotationAngle, modelRotationAxis.x, modelRotationAxis.y, modelRotationAxis.z);
    }

    if (mode == SHADED || mode == BOTH) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_LIGHTING);
        glColor3f(0.8f, 0.8f, 0.8f);

        glBegin(GL_TRIANGLES);
        for (auto f = mesh.facets_begin(); f != mesh.facets_end(); ++f) {
            auto h = f->facet_begin();
            CGAL_assertion(CGAL::circulator_size(h) == 3); // STL is triangulated

            do {
                const auto& p = h->vertex()->point();
                glVertex3f(p.x(), p.y(), p.z());
            } while (++h != f->facet_begin());
        }
        glEnd();
    }

    if (mode == WIREFRAME || mode == BOTH) {
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glColor3f(0.1f, 1.0f, 0.1f);

        glBegin(GL_TRIANGLES);
        for (auto f = mesh.facets_begin(); f != mesh.facets_end(); ++f) {
            auto h = f->facet_begin();
            do {
                const auto& p = h->vertex()->point();
                glVertex3f(p.x(), p.y(), p.z());
            } while (++h != f->facet_begin());
        }
        glEnd();

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    if (mode == VERTICES) {
        glDisable(GL_LIGHTING);
        glColor3f(1.0f, 1.0f, 1.0f);
        glPointSize(5.0f);

        glBegin(GL_POINTS);
        for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
            const auto& p = v->point();
            glVertex3f(p.x(), p.y(), p.z());
        }
        glEnd();
    }
    glPopMatrix();
}
//void differentViewsModes(RenderMode mode) {
//    glPushMatrix();
//
//    if (controlMode == MODEL_VIEW) {
//        glScalef(modelScale, modelScale, modelScale);
//        glTranslatef(modelOffset.x, modelOffset.y, modelOffset.z);
//        glRotatef(modelRotationAngle, modelRotationAxis.x, modelRotationAxis.y, modelRotationAxis.z);
//    }
//
//    bool drawShaded = (mode == SHADED || mode == BOTH);
//    bool drawWire = (mode == WIREFRAME || mode == BOTH);
//    bool drawVertices = (mode == VERTICES);
//
//    if (drawShaded || drawWire) {
//        if (drawShaded) {
//            glEnable(GL_LIGHTING);
//            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//            glColor3f(0.8f, 0.8f, 0.8f);
//        }
//        else {
//            glDisable(GL_LIGHTING);
//            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//            glColor3f(0.1f, 1.0f, 0.1f);
//        }
//
//        glBegin(GL_TRIANGLES);
//        for (auto f = mesh.facets_begin(); f != mesh.facets_end(); ++f) {
//            auto h = f->facet_begin();
//            CGAL_assertion(CGAL::circulator_size(h) == 3);
//            do {
//                const auto& p = h->vertex()->point();
//                glVertex3f(p.x(), p.y(), p.z());
//            } while (++h != f->facet_begin());
//        }
//        glEnd();
//
//        // Reset fill mode if wireframe was drawn
//        if (drawWire) {
//            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//        }
//    }
//
//    if (drawVertices) {
//        glDisable(GL_LIGHTING);
//        glColor3f(1.0f, 1.0f, 1.0f);
//        glPointSize(5.0f);
//
//        glBegin(GL_POINTS);
//        for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
//            const auto& p = v->point();
//            glVertex3f(p.x(), p.y(), p.z());
//        }
//        glEnd();
//    }
//
//    glPopMatrix();
//}

//void drawSelectedRegion() {
//    glPushMatrix();
//    if ((show_handles || show_fixed) && (!handle_indices.empty() || !fixed_indices.empty())) {
//        glDisable(GL_LIGHTING);
//
//        glPointSize(5.0f);
//
//        glBegin(GL_POINTS);
//
//        for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
//            const auto& p = v->point();
//
//            if (show_handles && !handle_indices.empty() && handle_indices.count(v->id())) {
//
//                glColor3f(1.0f, 1.0f, 0.0f);
//                glVertex3f(p.x(), p.y(), p.z());
//
//            }
//
//            if (show_fixed && !fixed_indices.empty() && fixed_indices.count(v->id())) {
//
//                glColor3f(0.0f, 1.0f, 1.0f);
//                glVertex3f(p.x(), p.y(), p.z());
//
//            }
//        }
//        glEnd();
//    }
//    glPopMatrix();
//}
void drawSelectedRegion() {
    glPushMatrix();

    glDisable(GL_LIGHTING);
    glPointSize(5.0f);

    // Draw handle and fixed points
    glBegin(GL_POINTS);

    glm::vec3 minPt(FLT_MAX), maxPt(-FLT_MAX);

    for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
        const auto& pt = v->point();

        if ((show_handles || show_fixed) && (!handle_indices.empty() || !fixed_indices.empty())) {
            if (show_handles && handle_indices.count(v->id())) {
                glColor3f(1.0f, 1.0f, 0.0f);  // Yellow for handles
                glVertex3f(pt.x(), pt.y(), pt.z());
            }

            if (show_fixed && fixed_indices.count(v->id())) {
                glColor3f(0.0f, 1.0f, 1.0f);  // Cyan for fixed
                glVertex3f(pt.x(), pt.y(), pt.z());
            }
        }

        // Update bounding box only for handle points
        if (handle_indices.count(v->id())) {
            glm::vec3 p(pt.x(), pt.y(), pt.z());
            minPt = glm::min(minPt, p);
            maxPt = glm::max(maxPt, p);
        }
    }

    glEnd();  // End GL_POINTS

    // Draw bounding box if conditions are met
    if (mouseControlledDeform && boundingBoxVisible && minPt != glm::vec3(FLT_MAX)) {
        boundingBoxVisible = true;
        bboxCenter = 0.5f * (minPt + maxPt);

        glLineWidth(2.0f);
        glColor3f(1.0f, 0.0f, 0.0f);  // Red bounding box
        glBegin(GL_LINES);

        glm::vec3 min = minPt;
        glm::vec3 max = maxPt;

        // Define the 8 corners of the box
        glm::vec3 v0(min.x, min.y, min.z);
        glm::vec3 v1(max.x, min.y, min.z);
        glm::vec3 v2(max.x, max.y, min.z);
        glm::vec3 v3(min.x, max.y, min.z);
        glm::vec3 v4(min.x, min.y, max.z);
        glm::vec3 v5(max.x, min.y, max.z);
        glm::vec3 v6(max.x, max.y, max.z);
        glm::vec3 v7(min.x, max.y, max.z);

        // Bottom face
        glVertex3fv(glm::value_ptr(v0)); glVertex3fv(glm::value_ptr(v1));
        glVertex3fv(glm::value_ptr(v1)); glVertex3fv(glm::value_ptr(v2));
        glVertex3fv(glm::value_ptr(v2)); glVertex3fv(glm::value_ptr(v3));
        glVertex3fv(glm::value_ptr(v3)); glVertex3fv(glm::value_ptr(v0));

        // Top face
        glVertex3fv(glm::value_ptr(v4)); glVertex3fv(glm::value_ptr(v5));
        glVertex3fv(glm::value_ptr(v5)); glVertex3fv(glm::value_ptr(v6));
        glVertex3fv(glm::value_ptr(v6)); glVertex3fv(glm::value_ptr(v7));
        glVertex3fv(glm::value_ptr(v7)); glVertex3fv(glm::value_ptr(v4));

        // Vertical edges
        glVertex3fv(glm::value_ptr(v0)); glVertex3fv(glm::value_ptr(v4));
        glVertex3fv(glm::value_ptr(v1)); glVertex3fv(glm::value_ptr(v5));
        glVertex3fv(glm::value_ptr(v2)); glVertex3fv(glm::value_ptr(v6));
        glVertex3fv(glm::value_ptr(v3)); glVertex3fv(glm::value_ptr(v7));

        glEnd();
    }

    glEnable(GL_LIGHTING);
    glPopMatrix();
}
void drawSelectionPloygon() {
    glPushMatrix();

    // Draw selection polygon in 2D overlay
    if (!selectionPolygon.empty()) {
        glDisable(GL_LIGHTING);
        int width, height;
        glfwGetFramebufferSize(glfwGetCurrentContext(), &width, &height);

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, width, height, 0, -1, 1);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();

        glColor3f(1.0f, 1.0f, 0.0f);
        glBegin(GL_LINE_LOOP);
        for (const auto& pt : selectionPolygon) {
            glVertex2f(pt.x, pt.y);
        }
        glEnd();

        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
    }
    glPopMatrix();
}
void drawMouseControlledDeform() {
    glPushMatrix();
    if (mouseControlledDeform) {
        glm::vec3 minPt(FLT_MAX), maxPt(-FLT_MAX);

        for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
            if (handle_indices.count(v->id())) {
                const auto& pt = v->point();
                glm::vec3 p(pt.x(), pt.y(), pt.z());
                minPt = glm::min(minPt, p);
                maxPt = glm::max(maxPt, p);
            }
        }

        bboxCenter = 0.5f * (minPt + maxPt);
        boundingBoxVisible = true;
        glm::vec3 bboxMin = minPt;
        glm::vec3 bboxMax = maxPt;

        if (boundingBoxVisible) {
            glDisable(GL_LIGHTING);
            glLineWidth(2.0f);
            glColor3f(1.0f, 0.0f, 0.0f);  // Red bounding box

            glBegin(GL_LINES);

            glm::vec3 min = bboxMin;
            glm::vec3 max = bboxMax;

            // 8 corners
            glm::vec3 v0(min.x, min.y, min.z);
            glm::vec3 v1(max.x, min.y, min.z);
            glm::vec3 v2(max.x, max.y, min.z);
            glm::vec3 v3(min.x, max.y, min.z);
            glm::vec3 v4(min.x, min.y, max.z);
            glm::vec3 v5(max.x, min.y, max.z);
            glm::vec3 v6(max.x, max.y, max.z);
            glm::vec3 v7(min.x, max.y, max.z);

            // Bottom square
            glVertex3fv(glm::value_ptr(v0)); glVertex3fv(glm::value_ptr(v1));
            glVertex3fv(glm::value_ptr(v1)); glVertex3fv(glm::value_ptr(v2));
            glVertex3fv(glm::value_ptr(v2)); glVertex3fv(glm::value_ptr(v3));
            glVertex3fv(glm::value_ptr(v3)); glVertex3fv(glm::value_ptr(v0));

            // Top square
            glVertex3fv(glm::value_ptr(v4)); glVertex3fv(glm::value_ptr(v5));
            glVertex3fv(glm::value_ptr(v5)); glVertex3fv(glm::value_ptr(v6));
            glVertex3fv(glm::value_ptr(v6)); glVertex3fv(glm::value_ptr(v7));
            glVertex3fv(glm::value_ptr(v7)); glVertex3fv(glm::value_ptr(v4));

            // Vertical lines
            glVertex3fv(glm::value_ptr(v0)); glVertex3fv(glm::value_ptr(v4));
            glVertex3fv(glm::value_ptr(v1)); glVertex3fv(glm::value_ptr(v5));
            glVertex3fv(glm::value_ptr(v2)); glVertex3fv(glm::value_ptr(v6));
            glVertex3fv(glm::value_ptr(v3)); glVertex3fv(glm::value_ptr(v7));

            glEnd();
            glEnable(GL_LIGHTING);
        }

    }
    glPopMatrix();
}
void drawModel(RenderMode mode)  {
    differentViewsModes(mode);
    drawSelectionPloygon();
    drawSelectedRegion();
   // drawMouseControlledDeform();
}
//////////////////////////////////////////////////////////////////////////////////Model rendering/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////Mesh smooth/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool is_border_vertex(Polyhedron::Vertex& v) {
    HV_circulator h = v.vertex_begin();
    HV_circulator end = h;

    if (h == NULL) return false; // isolated vertex

    do {
        if (h->is_border()) {
            return true;
        }
        ++h;
    } while (h != end);

    return false;
}
void laplacian_smooth(Polyhedron& mesh, const std::set<int>& fixed_vertex_ids, int iterations = 10) {// Laplacian smoothing with fixed vertex 
    for (int iter = 0; iter < iterations; ++iter) {
        std::map<Vertex_iterator, Point> new_positions;

        for (Vertex_iterator v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) {
            int vid = v->id();

            // skip border or fixed vertices
            //if (fixed_vertex_ids.count(vid) || is_border_vertex(*v))
            //    continue;
            if (is_border_vertex(*v))
                continue;

            HV_circulator h = v->vertex_begin();
            HV_circulator end = h;
            int count = 0;
            Kernel::Vector_3 avg(0, 0, 0);

            CGAL_For_all(h, end) {
                Point neighbor = h->opposite()->vertex()->point();
                avg = avg + (neighbor - CGAL::ORIGIN);
                ++count;
            }

            if (count > 0) {
                Point smoothed = CGAL::ORIGIN + (avg / count);
                new_positions[v] = smoothed;
            }
        }

        // apply updated positions
        for (const auto& pair : new_positions) {
            pair.first->point() = pair.second;
        }
    }
}
void smooth_after_bend(Polyhedron& mesh, const std::set<int>& fixed_vertex_ids, int subdivision_level = 1, int smoothing_iters = 5) {
    //  subdivision to increase resolution
    std::cout << "before Subdivision method3 Loop_subdivision has : " << mesh.size_of_vertices() << " vertices, " << mesh.size_of_facets() << " triangles\n";
   CGAL::Subdivision_method_3::Loop_subdivision(mesh, subdivision_level);

    std::cout << "after Subdivision method3 Loop_subdivision has : " << mesh.size_of_vertices() << " vertices, " << mesh.size_of_facets() << " triangles\n";
    // CGAL::Subdivision_method_3::CatmullClark_subdivision(mesh, CGAL::parameters::number_of_iterations(3));
    mesh.normalize_border();
    
    //  Laplacian smoothing
    laplacian_smooth(mesh, fixed_vertex_ids, smoothing_iters);
}
//////////////////////////////////////////////////////////////////////////////////Mesh smooth/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
