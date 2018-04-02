#include "tzj_slam/transform.hpp"


Vector3d world2camera ( const Vector3d& p_w, const SE3& T_c_w )
{
    return T_c_w*p_w;
}

Vector3d camera2world ( const Vector3d& p_c, const SE3& T_c_w )
{
    return T_c_w.inverse() *p_c;
}

Vector2d camera2pixel ( const Vector3d& p_c, double fx_, double cx_, double fy_, double cy_ )
{
    return Vector2d (
               fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,
               fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_
           );
}

Vector3d pixel2camera ( const Vector2d& p_p, double depth, double fx_, double cx_, double fy_, double cy_)
{
    return Vector3d (
               ( p_p ( 0,0 )-cx_ ) *depth/fx_,
               ( p_p ( 1,0 )-cy_ ) *depth/fy_,
               depth
           );
}

Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w, double fx_, double cx_, double fy_, double cy_ )
{
    return camera2pixel ( world2camera(p_w, T_c_w), fx_, cx_, fy_, cy_ );
}

Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth, double fx_, double cx_, double fy_, double cy_ )
{
    return camera2world ( pixel2camera ( p_p, depth, fx_, cx_, fy_, cy_ ), T_c_w );
}