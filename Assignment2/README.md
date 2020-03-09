#### Inside triangle

``` cpp
static bool insideTriangle(int x, int y, const Vector3f* _v)
{   

    for (int i = 0; i < 3; ++ i) {
        float xx = 1.0 * x - _v[i].x();
        float yy = 1.0 * y - _v[i].y();
        auto vv = _v[(i+1)%3] - _v[i];

        float tmp = vv.x() * yy - vv.y() * xx;
        if(tmp < 0) 
            return false;
    }
    return true;
    
}
```

#### Get Bounding box
``` cpp
float l = width, r = 0, d = height, t = 0;
for (int i = 0; i < 3; ++ i) {
    l = std::min(l, v[i][0]);
    r = std::max(r, v[i][0]);
    d = std::min(d, v[i][1]);
    t = std::max(t, v[i][1]);
}
for (int x = l; x < r; ++ x) {
    for (int y = d; y < t; ++ y) {

    }
}
```

#### Judge inside and calculate z-depth
``` cpp
if(insideTriangle(x, y, tri.v)) {
    
    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, tri.v);
    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    z_interpolated *= w_reciprocal;

    Eigen::Vector3f point = Eigen::Vector3f(x, y, z_interpolated);
    set_pixel(point, tri.getColor());
}
```


#### Fill the pixel with color
``` cpp
void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    auto ind = (height-1-point.y())*width + point.x();
    
    if(depth_buf[ind] > point.z()) {
        depth_buf[ind] = point.z();
        frame_buf[ind] = color;
    }

}

```