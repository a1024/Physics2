#undef			UNICODE
#include		<Windows.h>
#include		<stdio.h>
#include		<math.h>
#include		<time.h>
#include		<vector>
#include		<queue>
#include		<string>
#include		<sstream>
	#define		LOCAL_COORDINATES
	#define		ITERATIONS
//	#define		ENGINE_V4
	#define		ENGINE_V5
//	#define		ENGINE_V3
int				w=0, h=0, Ox, Oy;
bool			timer=false, tick=false, pause=false, info=true, rotational_pause=false;

int				scenario_number=0, frame_number=0;
double			timescale=0.5;//1 0.5 0.1	0.05 0.005
#if defined ENGINE_V4||defined ENGINE_V5
double			beta=0.2,//bias factor [0.1, 0.3]
				C_R=0.4,//coefficient of restitution	0.8
				v_convergence=0.01,//velocity convergence threshold
				penetration_slop=0.5,//allowed penetration	0.01 X
				restitution_slop=1,

				object_friction=0.4,//0.2 0.4	X 0.5 jumps
				boundary_friction=0.4,

				proximity_limit=5,//0.1 X		to old contact points
				old_lambda_factor=0.6,
				min_dp=0.01, min_da=0.001;//minimum allowed change in position and angle per frame
int				n_iterations=10;//better friction 10	100
	bool		warm_starting=true,//accumulating points
//	bool		warm_starting=false,
				collision_detection_SAT=true,
				sleeping_enabled=false;
#endif
#ifdef ENGINE_V5
bool			accumulated_impulses=false;
#endif
double			air_viscosity=1,//1 0.98
				damping=0.8;//0.1 jelly	0.5 on ground	0.7 attenuate		0.8 ground bounce	0.95 increase	0.978260869565218		1 increase

int				xoffset=100, yoffset=100;//10	100, 200
double			yground=0, ytop=50*5, xleft=0, xright=120*5, gravity=0.98;//0.98	0.098*5;	0
//double		yground=0, gravity=0.098;
//double		xleft=0, xright=120, ytop=50;

int				jumpStart=0, jumpDuration=500;
double			jumpVelocity=4,//2 [4] 6
				runForce=1,//0.1 0.2 0.4 1
				runLimit=10;//4 10 20
double			lastlambda=0, lastlambda_walls=0;

HWND			ghWnd;
HDC				ghDC, ghMemDC;
HBITMAP			ghBitmap;
bool			kb[256]={0};
const double	pi=acos(-1.), pi2=2*pi;
double			inv_sqrt(double x)//http://stackoverflow.com/questions/11513344/how-to-implement-the-fast-inverse-square-root-in-java
{
	double t0;
	(long long&)t0=0x5FE6EC85E7DE30DA-((long long&)x>>1);
	return t0*(1.5-.5*x*t0*t0);
}
const int		g_bufsize=1024;
int				g_buflen;
char			g_buf[g_bufsize];
void			GUIPrint(HDC__ *hDC, int x, int y, const char* a, ...)
{
	g_buflen=vsprintf_s(g_buf, g_bufsize, a, (char*)(&a+1));
//	g_buflen=vsprintf_s(g_buf, g_bufsize, a, (char*)(&reinterpret_cast<const char &>(a))+((sizeof(a)+3)&~3));
	if(g_buflen>0)
		TextOutA(hDC, x, y, g_buf, g_buflen);
}
void			GUIPrint(HDC__ *hDC, int x, int y, int value)
{
	g_buflen=sprintf_s(g_buf, 1024, "%d", value);
	if(g_buflen>0)
		TextOutA(hDC, x, y, g_buf, g_buflen);
}
void			GUIPrint(HDC__ *hDC, double x, double y, const char* a, ...)
{
	g_buflen=vsprintf_s(g_buf, g_bufsize, a, (char*)(&a+1));
	if(g_buflen>0)
		TextOutA(hDC, xoffset+int(x)-(x<0), h-yoffset-(int(y)-(y<0)), g_buf, g_buflen);
}
void			GUIPrint(HDC__ *hDC, double x, double y, int value)
{
	g_buflen=sprintf_s(g_buf, 1024, "%d", value);
	if(g_buflen>0)
		TextOutA(hDC, xoffset+int(x)-(x<0), h-yoffset-(int(y)-(y<0)), g_buf, g_buflen);
}
double			mod(double x, double y){return x-y*floor(x/y);}
/*void			adjust_angle(double &angle)
{
	double angle_pi2=angle/pi2;
	angle=pi2*(angle_pi2-floor(angle_pi2));
}//*/
double			adjust_angle(double angle)//reduce angle to the range [0, 2pi[
{
	double angle_pi2=angle/pi2;
	angle=pi2*(angle_pi2-floor(angle_pi2));
	if(angle<0||angle>=pi2)
		return 0;
	return pi2*(angle_pi2-floor(angle_pi2));
}
double			intersection_lines_nonparallel_x(double m1x, double m1y, double ax1, double ay1, double m2x, double m2y, double bx1, double by1)
{
	double temp1=m2x*m1y, temp2=m1x*m2y;
	return (temp1*ax1 - temp2*bx1 + m2x*m1x*(by1-ay1))/(temp1-temp2);
}
/*bool			intersection_lines(double ax1, double ay1, double ax2, double ay2, double bx1, double by1, double bx2, double by2, double &x, double &y)
{
	m1x=ax2-ax1, m1y=ay2-ay1;
	m2x=bx2-bx1, m2y=by2-by1;
	temp1=m2x*m1y, temp2=m1x*m2y, cross=temp1-temp2;
	if(cross)//not parallel
	{
		x=(temp1*ax1 - temp2*bx1 + m2x*m1x*(by1-ay1))/cross;
		y=m1x ? m1y/m1x*(x-ax1)+ay1
			: m2y/m2x*(x-bx1)+by1;//a is vertical
		return true;
	}
	if(abs(m1x)>abs(m1y))
	{
		x=bx1, y=m1y/m1x*(x-ax1)+ay1;
		return y==by1;
	}
	y=by1, x=m1x/m1y*(y-ay1)+ax1;
	return x==bx1;
}//*/
/*bool intersection_lines(double m1x, double m1y, double ax1, double ay1, double m2x, double m2y, double bx1, double by1, double &x, double &y)
{
	double temp1=m2x*m1y, temp2=m1x*m2y, cross=temp1-temp2;
	if(cross)//not parallel
	{
		x=(temp1*ax1 - temp2*bx1 + m2x*m1x*(by1-ay1))/cross;
		y=m1x ? m1y/m1x*(x-ax1)+ay1
			: m2y/m2x*(x-bx1)+by1;//a is vertical
		return true;
	}
	if(abs(m1x)>abs(m1y))
	{
		x=bx1, y=m1y/m1x*(x-ax1)+ay1;
		return y==by1;
	}
	y=by1, x=m1x/m1y*(y-ay1)+ax1;
	return x==bx1;
}//*/
/*void intersection_lines_nonparallel(double ax1, double ay1, double ax2, double ay2, double bx1, double by1, double bx2, double by2, double &x, double &y)
{
	double
		m1x=ax2-ax1, m1y=ay2-ay1,
		m2x=bx2-bx1, m2y=by2-by1,
		temp1=m2x*m1y, temp2=m1x*m2y;
	x=(temp1*ax1 - temp2*bx1 + m2x*m1x*(by1-ay1))/(temp1-temp2);
	y=m1x ? m1y/m1x*(x-ax1)+ay1
		: m2y/m2x*(x-bx1)+by1;//a is vertical
}
void intersection_lines_nonparallel(double m1x, double m1y, double ax1, double ay1, double m2x, double m2y, double bx1, double by1, double &x, double &y)
{
	double temp1=m2x*m1y, temp2=m1x*m2y;
	x=(temp1*ax1 - temp2*bx1 + m2x*m1x*(by1-ay1))/(temp1-temp2);
	y=m1x ? m1y/m1x*(x-ax1)+ay1
		: m2y/m2x*(x-bx1)+by1;//a is vertical
}//*/
/*bool intersection_segments(double ax1, double ay1, double ax2, double ay2, double bx1, double by1, double bx2, double by2, double &x, double &y)
{
	double
		m1x=ax2-ax1, m1y=ay2-ay1,
		m2x=bx2-bx1, m2y=by2-by1;
	double temp1=m2x*m1y, temp2=m1x*m2y, cross=temp1-temp2;
	if(cross)//not parallel
	{
		x=(temp1*ax1 - temp2*bx1 + m2x*m1x*(by1-ay1))/cross;
		y=m1x ? m1y/m1x*(x-ax1)+ay1
			: m2y/m2x*(x-bx1)+by1;//a is vertical

		//check if (x,y) inside segments a and b
		if(abs(m1x)>abs(m1y))
			return (ax1<ax2?ax1<x&&x<ax2:ax2<x&&x<ax1)&&(bx1<bx2?bx1<x&&x<bx2:bx2<x&&x<bx1);
		return (ay1<ay2?ay1<y&&y<ay2:ay2<y&&y<ay1)&&(by1<by2?by1<y&&y<by2:by2<y&&y<by1);
	}
	//parallel
	double *axx1, *ayy1, *axx2, *ayy2, *bxx1, *byy1, *bxx2, *byy2, *xx, *yy, m1;
	if(abs(m1x)>abs(m1y))
		axx1=&ax1, ayy1=&ay1, axx2=&ax2, ayy2=&ay2, bxx1=&bx1, byy1=&by1, bxx2=&bx2, byy2=&by2, xx=&x, yy=&y, m1=m1y/m1x;
	else
		axx1=&ay1, ayy1=&ax1, axx2=&ay2, ayy2=&ax2, bxx1=&by1, byy1=&bx1, bxx2=&by2, byy2=&bx2, xx=&y, yy=&x, m1=m1x/m1y;
	double ya=m1y/m1x*(*bxx1-*axx1)+*ayy1;
	if(ya==by1)//collinear
	{
		double *ax_a, *ay_a, *ax_b, *ay_b, *bx_a, *by_a, *bx_b, *by_b;
		if(ax1<ax2)
			ax_a=axx1, ay_a=ayy1, ax_b=axx2, ay_b=ayy2;
		else
			ax_a=axx2, ay_a=ayy2, ax_b=axx1, ay_b=ayy1;
		if(bx1<bx2)
			bx_a=axx1, by_a=ayy1, bx_b=axx2, by_b=ayy2;
		else
			bx_a=axx2, by_a=ayy2, bx_b=axx1, by_b=ayy1;
		if(*ax_a<*bx_b&&*ax_b>*bx_a)//intersect
		{
				 if(*ax_a<*bx_a&&*bx_b<*ax_b)	*xx=(*bx_a+*bx_b)/2, *yy=(*by_a+*by_b)/2;//abba
			else if(*ax_a<*bx_a&&*ax_b<*bx_b)	*xx=(*bx_a+*ax_b)/2, *yy=(*by_a+*ay_b)/2;//abab
			else if(*bx_a<*ax_a&&*bx_b<*ax_b)	*xx=(*ax_a+*bx_b)/2, *yy=(*ay_a+*by_b)/2;//baba
			else								*xx=(*ax_a+*ax_b)/2, *yy=(*ay_a+*ay_b)/2;//baab
		//	else if(*bx_a<*ax_a&&*ax_b<*bx_b)	*xx=(*ax_a+*ax_b)/2, *yy=(*ay_a+*ay_b)/2;//baab
			return true;
		}
	}
	return false;
}//*/
struct			Point
{
	double x, y;
	Point():x(0), y(0){}
	Point(Point const &p):x(p.x), y(p.y){}
	Point(double x, double y):x(x), y(y){}
	void set(double x, double y){this->x=x, this->y=y;}
	Point& operator+=(Point const &b){x+=b.x, y+=b.y; return *this;}
	Point& operator-=(Point const &b){x-=b.x, y-=b.y; return *this;}
	Point& operator+=(double x){this->x+=x, y+=x; return *this;}
	Point& operator-=(double x){this->x-=x, y-=x; return *this;}
	Point& operator*=(double x){this->x*=x, y*=x; return *this;}
	Point& operator/=(double x){this->x/=x, y/=x; return *this;}
	double dot(Point const &other)const{return x*other.x+y*other.y;}
	double cross(Point const &other)const{return x*other.y-y*other.x;}
	Point cross(double z){return Point(z*y, -z*x);}
	double magnitude()const{return sqrt(x*x+y*y);}
	double mag_sq()const{return x*x+y*y;}
	double angle()const{return atan(y/x);}
	double angle2()const{return atan2(y, x);}
};
inline Point	operator*(Point const &p, double x){return Point(p.x*x, p.y*x);}
inline Point	operator*(double x, Point const &p){return Point(p.x*x, p.y*x);}
inline Point	operator/(Point const &p, double x){return Point(p.x/x, p.y/x);}
inline Point	operator+(Point const &a, Point const &b){return Point(a.x+b.x, a.y+b.y);}
inline Point	operator-(Point const &a, Point const &b){return Point(a.x-b.x, a.y-b.y);}
inline Point	operator+(Point const &p, double x){return Point(p.x+x, p.y+x);}
inline Point	operator-(Point const &p, double x){return Point(p.x-x, p.y-x);}
inline bool		operator==(Point const &a, Point const &b){return a.x==b.x&&a.y==b.y;}
inline bool		operator!=(Point const &a, Point const &b){return a.x!=b.x||a.y!=b.y;}
inline Point	operator-(Point const &p){return Point(-p.x, -p.y);}
inline Point	cross(double z, Point const &p){return Point(-z*p.y, z*p.x);}
inline void		draw_line(Point const &a, Point const &b, int line_color)
{
	HPEN hPen=CreatePen(PS_SOLID, 1, line_color);
	hPen=(HPEN)SelectObject(ghMemDC, hPen);
	MoveToEx(ghMemDC, xoffset+int(a.x)-(a.x<0), h-yoffset-int(a.y)-(a.y<0), 0);
	LineTo(ghMemDC, xoffset+int(b.x)-(b.x<0), h-yoffset-int(b.y)-(b.y<0));
	hPen=(HPEN)SelectObject(ghMemDC, hPen);
	DeleteObject(hPen);
}
inline void		draw_line(Point const &a, Point const &b)
{
	MoveToEx(ghMemDC, xoffset+int(a.x)-(a.x<0), h-yoffset-int(a.y)-(a.y<0), 0);
	LineTo(ghMemDC, xoffset+int(b.x)-(b.x<0), h-yoffset-int(b.y)-(b.y<0));
}
inline void		draw_line(double x1, double y1, double x2, double y2)
{
	MoveToEx(ghMemDC, xoffset+int(x1)-(x1<0), h-yoffset-int(y1)-(y1<0), 0);
	LineTo(ghMemDC, xoffset+int(x2)-(x2<0), h-yoffset-int(y2)-(y2<0));
}
inline void		draw_rectangle(double x1, double x2, double y1, double y2)//BL-TR
{
	int X1=xoffset+int(x1)-(x1<0), X2=xoffset+int(x2)-(x2<0),
		Y1=h-yoffset-(int(y1)-(y1<0)), Y2=h-yoffset-(int(y2)-(y2<0));
	MoveToEx(ghMemDC, X1, Y1, 0);
	LineTo	(ghMemDC, X1, Y2);
	MoveToEx(ghMemDC, X1, Y2, 0);
	LineTo	(ghMemDC, X2, Y2);
	MoveToEx(ghMemDC, X2, Y2, 0);
	LineTo	(ghMemDC, X2, Y1);
	MoveToEx(ghMemDC, X2, Y1, 0);
	LineTo	(ghMemDC, X1, Y1);
}
inline void		draw_rectangle(Point const &a, Point const &b){draw_rectangle(a.x, b.x, a.y, b.y);}
inline void		draw_ellipse(Point const &BL, Point const &TR)
{
	HBRUSH hBrush=(HBRUSH)GetStockObject(HOLLOW_BRUSH);
	hBrush=(HBRUSH)SelectObject(ghMemDC, hBrush);
	Ellipse(ghMemDC, xoffset+int(BL.x)-(BL.x<0), h-yoffset-int(BL.y)-(BL.y<0), xoffset+int(TR.x)-(TR.x<0), h-yoffset-int(TR.y)-(TR.y<0));
	hBrush=(HBRUSH)SelectObject(ghMemDC, hBrush);
}
//double		interpolation_line_nonvertical_y(double x1, double y1, double x2, double y2, double x){return (y2-y1)/(x2-x1)*(x-x1)+y1;}
double			interpolation_line_nonvertical_y(Point const &p1, Point const &p2, double x)
{
	return p1.y+(p2.y-p1.y)/(p2.x-p1.x)*(x-p1.x);
}
inline void		sort_coordinates(Point const &back, Point const &front, Point &BL, Point &TR)
{
	double x_sum=back.x+front.x, x_dif=abs(back.x-front.x), y_sum=back.y+front.y, y_dif=abs(back.y-front.y);
	BL.set(0.5*(x_sum-x_dif), 0.5*(y_sum-y_dif));
	TR.set(0.5*(x_sum+x_dif), 0.5*(y_sum+y_dif));
}
inline void		sort_coordinates(Point &BL, Point &TR)
{
	double x_sum=BL.x+TR.x, x_dif=abs(BL.x-TR.x), y_sum=BL.y+TR.y, y_dif=abs(BL.y-TR.y);
	BL.set(0.5*(x_sum-x_dif), 0.5*(y_sum-y_dif));
	TR.set(0.5*(x_sum+x_dif), 0.5*(y_sum+y_dif));
}

struct			RotatedRectangle
{
	Point center, size;
	double angle, cos_a, sin_a;
	RotatedRectangle():cos_a(0), sin_a(0){}
	RotatedRectangle(Point const &center, Point const &size, double cos_a, double sin_a):center(center), size(size), cos_a(cos_a), sin_a(sin_a){}
	void set(Point const &center, Point const &size, double cos_a, double sin_a){this->center=center, this->size=size, this->cos_a=cos_a, this->sin_a=sin_a;}
	void draw()//
	{
		Point size_2=size*0.5, arrow(cos_a, sin_a), arrow_n(sin_a, -cos_a);
		Point p1=center-arrow*size_2.x-arrow_n*size_2.y,
			p2=center-arrow*size_2.x+arrow_n*size_2.y,
			p3=center+arrow*size_2.x+arrow_n*size_2.y,
			p4=center+arrow*size_2.x-arrow_n*size_2.y;
		draw_line(p1, p2), draw_line(p2, p3), draw_line(p3, p4), draw_line(p4, p1);
		//draw_line(p1-Point(5, 0), p1+Point(5, 0)), draw_line(p1-Point(0, 5), p1+Point(0, 5));
		//draw_line(p2-Point(5, 0), p2+Point(5, 0)), draw_line(p2-Point(0, 5), p2+Point(0, 5));
		//draw_line(p3-Point(5, 0), p3+Point(5, 0)), draw_line(p3-Point(0, 5), p3+Point(0, 5));
		//draw_line(p4-Point(5, 0), p4+Point(5, 0)), draw_line(p4-Point(0, 5), p4+Point(0, 5));
		//GUIPrint(ghMemDC, p1.x, p1.y, 1);
		//GUIPrint(ghMemDC, p2.x, p2.y, 2);
		//GUIPrint(ghMemDC, p3.x, p3.y, 3);
		//GUIPrint(ghMemDC, p4.x, p4.y, 4);
	}
};
inline void		RotateVector2DClockwise(Point &v, double ang)
{
	double t, cosa=cos(ang), sina=sin(ang);
	t = v.x,
		v.x=t*cosa+v.y*sina,
		v.y=-t*sina+v.y*cosa;
}
bool			rotated_rectangle_intersect(RotatedRectangle const &rr1, RotatedRectangle const &rr2)//https://www.ragestorm.net/tutorial?id=22
{
	Point A, B, //vertices of the rotated rr2
		C,      //center of rr2
		BL, TR; //vertices of rr2 (bottom-left, top-right)

	double ang = rr1.angle - rr2.angle,			//orientation of rotated rr1
		cosa = cos(ang), sina = sin(ang);	//precalculated trigonometic values for repeated use

	double t, x, a;		//temporary variables for various uses
	double dx;			//deltaX for linear equations
	double ext1, ext2;	//min/max vertical values

	//move rr2 to make rr1 cannonic
	C=rr2.center;
	C-=rr1.center;//SubVectors2D(&C, &rr1->C);

	// rotate rr2 clockwise by rr2->ang to make rr2 axis-aligned
	RotateVector2DClockwise(C, rr2.angle);

	// calculate vertices of (moved and axis-aligned := 'ma') rr2
	BL = TR = C;
	BL-=rr2.size;//SubVectors2D(&BL, &rr2.size);
	TR+=rr2.size;//AddVectors2D(&TR, &rr2.size);

	// calculate vertices of (rotated := 'r') rr1
	A.x = -rr1.size.y*sina; B.x = A.x; t = rr1.size.x*cosa; A.x += t; B.x -= t;
	A.y =  rr1.size.y*cosa; B.y = A.y; t = rr1.size.x*sina; A.y += t; B.y -= t;

	t = sina*cosa;

	// verify that A is vertical min/max, B is horizontal min/max
	if (t < 0)
	{
		t = A.x; A.x = B.x; B.x = t;
		t = A.y; A.y = B.y; B.y = t;
	}

	// verify that B is horizontal minimum (leftest-vertex)
	if (sina < 0) { B.x = -B.x; B.y = -B.y; }

	// if rr2(ma) isn't in the horizontal range of
	// colliding with rr1(r), collision is impossible
	if (B.x > TR.x || B.x > -BL.x) return false;

	// if rr1(r) is axis-aligned, vertical min/max are easy to get
	if (t == 0) {ext1 = A.y; ext2 = -ext1; }
	// else, find vertical min/max in the range [BL.x, TR.x]
	else
	{
	x = BL.x-A.x; a = TR.x-A.x;
	ext1 = A.y;
	// if the first vertical min/max isn't in (BL.x, TR.x), then
	// find the vertical min/max on BL.x or on TR.x
	if (a*x > 0)
	{
	dx = A.x;
	if (x < 0) { dx -= B.x; ext1 -= B.y; x = a; }
	else       { dx += B.x; ext1 += B.y; }
	ext1 *= x; ext1 /= dx; ext1 += A.y;
	}
  
	x = BL.x+A.x; a = TR.x+A.x;
	ext2 = -A.y;
	// if the second vertical min/max isn't in (BL.x, TR.x), then
	// find the local vertical min/max on BL.x or on TR.x
	if (a*x > 0)
	{
	dx = -A.x;
	if (x < 0) { dx -= B.x; ext2 -= B.y; x = a; }
	else       { dx += B.x; ext2 += B.y; }
	ext2 *= x; ext2 /= dx; ext2 -= A.y;
	}
	}

	// check whether rr2(ma) is in the vertical range of colliding with rr1(r) (for the horizontal range of rr2)
	return !((ext1 < BL.y && ext2 < BL.y) || (ext1 > TR.y && ext2 > TR.y));
}

//enum			CollisionType{STATIC_STATIC, STATIC_DYNAMIC, STATIC_HINGE, DYNAMIC_DYNAMIC, HINGE_DYNAMIC, HINGE_HINGE};
struct			ContactPoint
{
	bool old;
	Point ra, rb;
	double travel;
#ifdef ENGINE_V5
	double mass_n, mass_t, bias;
	double Pn, Pt;
	ContactPoint():old(false), Pn(0), Pt(0){}
	ContactPoint(Point const &ra, Point const &rb, double travel):old(false), Pn(0), Pt(0), ra(ra), rb(rb), travel(travel){}
#elif defined ENGINE_V4
	double lambda_n, lambda_t;
	ContactPoint():old(false), lambda_n(0), lambda_t(0){}
	ContactPoint(Point const &ra, Point const &rb, double travel):old(false), lambda_n(0), lambda_t(0), ra(ra), rb(rb), travel(travel){}
#endif
};
struct			ContactInfo
{
	int a_idx, b_idx;
#if defined ENGINE_V4||defined ENGINE_V5
	std::vector<ContactPoint> p;
	//std::vector<std::pair<Point, Point>> r;//ra, rb
	//Point ra, rb;
#elif defined ENGINE_V3
	bool others_side;
	int point, side;
	bool rotational;
	double travel;
#endif
	double nx, ny;//pointing out of A
	bool persistent;
	ContactInfo():b_idx(-1){}
#if defined ENGINE_V4||defined ENGINE_V5
	ContactInfo(int a_idx, int b_idx, double travel, Point const &ra, Point const &rb, double nx, double ny):a_idx(a_idx), b_idx(b_idx), p(1, ContactPoint(ra, rb, travel)), nx(nx), ny(ny), persistent(false){}
	//CollisionInfo(int idx, double travel, Point const &ra, Point const &rb, double nx, double ny):idx(idx), p(1, CollisionPoint(ra, rb, travel)), nx(nx), ny(ny){}
	//CollisionInfo(int idx, double travel, Point const &ra, Point const &rb, double nx, double ny):idx(idx), travel(travel), r(1, std::pair<Point, Point>(ra, rb)), nx(nx), ny(ny){}
	//CollisionInfo(int idx, double travel, Point const &ra, Point const &rb, double nx, double ny):idx(idx), travel(travel), ra(ra), rb(rb), nx(nx), ny(ny){}
	void add_cp(Point const &ra, Point const &rb, double travel){p.push_back(ContactPoint(ra, rb, travel));}
#elif defined ENGINE_V3
	CollisionInfo(int idx, bool others_side, int point, int side, double travel, bool rotational, double nx, double ny):idx(idx), others_side(others_side), point(point), side(side), travel(travel), rotational(rotational), nx(nx), ny(ny){}
#endif
};
std::vector<ContactInfo> contacts;
struct			PhysPolygon
{
	int idx;
#if defined ENGINE_V4||defined ENGINE_V5
	char mech_unstable;//'U': mechanically unstable (dynamic), 'S': settled (sleeping)
	//std::vector<CollisionInfo> collisions;
	//std::vector<CollisionInfo> collisions_above, collisions_below;
	double friction;
#elif defined ENGINE_V3
	char mech_unstable;//'U': mechanically unstable (dynamic), 'G': touching something(s) below, 'L': sliding, 'S': mechanically stable (static)
	std::vector<CollisionInfo> collisions, resting_on;//-2: boundary, -1: nothing, >=0: object index
#endif

	std::vector<Point> w,//world coordinates
		vv;//vertices with origin at centroid, angle=0
	double r,//reach (distance to farthest point from centroid)
		J, inv_J;//moment of inertia
	double density,//properties
		mass, inv_mass;

	//Point Fsum;//sum of all acting forces
	//double Tsum;//sum of all acting torques

	Point p;//position (centroid)
	double angle, ca, sa;//angle of vertex 1 with centroid
	//double px, py, angle,//position (centroid)
	//	ca, sa;//angle of vertex 1 with centroid
	Point v;//velocity vector
	double vr;//radial velocity
	//double vx, vy, vr;//velocity
	//PhysPolygon():mech_unstable('U'), Fsum(0, 0), Tsum(0){}
	//PhysPolygon(double a, double c, double th1, double density):mech_unstable('U'), Fsum(0, 0), Tsum(0){set_properties_triangle_ac(a, c, th1, density);}
	//PhysPolygon(double x1, double y1, double x2, double y2, double x3, double y3, double density):mech_unstable('U'), Fsum(0, 0), Tsum(0){set_properties_triangle_xy(x1, y1, x2, y2, x3, y3, density);}
	PhysPolygon():mech_unstable('U'){}
	PhysPolygon(double a, double c, double th1, double density):mech_unstable('U'){set_properties_triangle_ac(a, c, th1, density);}
	PhysPolygon(double x1, double y1, double x2, double y2, double x3, double y3, double density):mech_unstable('U'){set_properties_triangle_xy(x1, y1, x2, y2, x3, y3, density);}
	
	void iterate(double delta)
	{
		//v+=Fsum*inv_mass*delta;
		//vr+=Tsum*inv_J*delta;

		p+=v*delta;
		//px+=vx*delta, py+=vy*delta;
		angle=adjust_angle(angle+vr*delta);
		udpate_angle(), update_world_coord();
	}
	//void integrate(double delta)
	//{
	//	v+=Fsum*inv_mass*delta, v.y-=gravity*delta;
	//	vr+=Tsum*inv_J*delta;
	//}
	void sleep()
	{
		mech_unstable='S';
		v.set(0, 0), vr=0;
	}
	void rotate_around(Point const &pivot, double cos_a, double sin_a)
	{
		angle=adjust_angle(angle+atan2(sin_a, cos_a));
		udpate_angle();
		double dx=p.x-pivot.x, dy=p.y-pivot.y;
	//	double dx=px-pivot.x, dy=py-pivot.y;
		p.set(pivot.x+dx*cos_a-dy*sin_a, pivot.y+dy*cos_a+dx*sin_a);
		update_world_coord();
	}
	void rotate_around(Point const &pivot, double d_angle)
	{
		angle=adjust_angle(angle+d_angle);
		udpate_angle();
		double dx=p.x-pivot.x, dy=p.y-pivot.y, d=sqrt(dx*dx+dy*dy);
		//double dx=px-pivot.x, dy=py-pivot.y, d=sqrt(dx*dx+dy*dy);
		double angle2=atan2(dy, dx)+d_angle;
		p.x=pivot.x+d*cos(angle2), p.y=pivot.y+d*sin(angle2);
		//px=pivot.x+d*cos(angle2), py=pivot.y+d*sin(angle2);
		update_world_coord();
	}
	void position_at(int point, Point const &p, int side, double side_angle)
	{
		double dx=w[(side+1)%w.size()].x-w[side].x, dy=w[(side+1)%w.size()].y-w[side].y;
		double angle_change=side_angle-atan2(dy, dx);
		rotate_around(p, angle_change);
		//rotate_around(Point(px, py), angle_change);
		translate_world(p-w[point]);
	}
	void translate_world(Point const &d)
	{
		p+=d;
	//	px+=d.x, py+=d.y;
		for(int k=0, kEnd=w.size();k<kEnd;++k)
			w[k]+=d;
	}
	void move_world(double px, double py, double angle)
	{
		p.x=px, p.y=py, this->angle=angle;
		//this->px=px, this->py=py, this->angle=angle;
		udpate_angle(), update_world_coord();
	}
	void udpate_angle(){ca=cos(angle), sa=sin(angle);}
	Point local_to_world(Point const &p)
	{
		//	cos(a)	-sin(a)		x
		//	sin(a)	cos(a)		y
		return Point(this->p.x+ca*p.x-sa*p.y, this->p.y+sa*p.x+ca*p.y);
	}
	Point local_to_world_rotate_only(Point const &p)
	{
		return Point(ca*p.x-sa*p.y, sa*p.x+ca*p.y);
	}
	Point world_to_local(Point p)
	{
		//	cos(a)	sin(a)		x
		//	-sin(a)	cos(a)		y
		p-=this->p;
		return Point(ca*p.x+sa*p.y, -sa*p.x+ca*p.y);
	}
	void update_world_coord()
	{
		for(int k=0, kEnd=w.size();k<kEnd;++k)
			w[k].set(p.x+ca*vv[k].x-sa*vv[k].y, p.y+sa*vv[k].x+ca*vv[k].y);
			//w[k].x=px+ca*vv[k].x-sa*vv[k].y, w[k].y=py+sa*vv[k].x+ca*vv[k].y;
	}

	void set_properties_triangle_ac(double a, double c, double th1, double density)
	{
		vv.resize(3), w.resize(3);
		double costh1=cos(th1), sinth1=sin(th1);
		double X1=0, Y1=0, X2=-a*costh1, Y2=a*sinth1, X3=-c, Y3=0;			//temp coordinates
		double
			ma3=(Y3-(Y1+Y2)/2)/(X3-(X1+X2)/2), tX3=X3, tY3=Y3,//medians
			mc2=(Y2-(Y1+Y3)/2)/(X2-(X1+X3)/2), tX2=X2, tY2=Y2;
		if(ma3==_HUGE||ma3==-_HUGE)
			ma3=(Y1-(Y2+Y3)/2)/(X1-(X2+X3)/2), tX3=X1, tY3=Y1;//b1
		else if(mc2==_HUGE||ma3==-_HUGE)
			mc2=(Y1-(Y2+Y3)/2)/(X1-(X2+X3)/2), tX2=X1, tY2=Y1;//b1
		double
			px=(ma3*tX3-mc2*tX2+tY2-tY3)/(ma3-mc2),//centroid
			py=ma3*(px-tX3)+tY3;
	//	double
	//		xa=(X1+X2)/2, ya=(Y1+Y2)/2, ma3=(X3-xa)/(Y3-ya),
	//		xc=(X1+X3)/2, yc=(Y1+Y3)/2, mc2=(X2-xc)/(Y2-yc);
	//	double
	//		px=(ma3*x3-mc2*x2+y2-y3)/(ma3-mc2),//centroid
	//		py=ma3*(px-x3)+y3;
		X1-=px, Y1-=py, X2-=px, Y2-=py, X3-=px, Y3-=py;
		double t_hyp=sqrt(X1*X1+Y1*Y1), cta=X1/t_hyp, sta=Y1/t_hyp;
	//	double t_angle=atan2(Y1, X1), cta=cos(t_angle), sta=sin(t_angle);
		vv[0].x=cta*X1+sta*Y1, vv[0].y=-sta*X1+cta*Y1;
		vv[1].x=cta*X2+sta*Y2, vv[1].y=-sta*X2+cta*Y2;
		vv[2].x=cta*X3+sta*Y3, vv[2].y=-sta*X3+cta*Y3;

		r=X1*X1+Y1*Y1;				//reach
		double	dd=X2*X2+Y2*Y2;	if(r<dd)r=dd;
				dd=X3*X3+Y3*Y3;	if(r<dd)r=dd;
		r=sqrt(r);

		this->density=density;
		set_weight_triangle();
	}
	void set_properties_triangle_abc(double a, double b, double c, double density)
	{
		vv.resize(3), w.resize(3);
		double a2=a*a, b2=b*b, c2=c*c;
		double costh1=(a2+b2-c2)/(2*a*b), th1=acos(costh1), sinth1=sin(th1);//th<180
		double X1=0, Y1=0, X2=-a*costh1, Y2=a*sinth1, X3=-c, Y3=0;			//temp coordinates
		double
			ma3=(Y3-(Y1+Y2)/2)/(X3-(X1+X2)/2), tX3=X3, tY3=Y3,//medians
			mc2=(Y2-(Y1+Y3)/2)/(X2-(X1+X3)/2), tX2=X2, tY2=Y2;
		if(ma3==_HUGE||ma3==-_HUGE)
			ma3=(Y1-(Y2+Y3)/2)/(X1-(X2+X3)/2), tX3=X1, tY3=Y1;//b1
		else if(mc2==_HUGE||ma3==-_HUGE)
			mc2=(Y1-(Y2+Y3)/2)/(X1-(X2+X3)/2), tX2=X1, tY2=Y1;//b1
		double
			px=(ma3*tX3-mc2*tX2+tY2-tY3)/(ma3-mc2),//centroid
			py=ma3*(px-tX3)+tY3;
	//	double
	//		xa=(X1+X2)/2, ya=(Y1+Y2)/2, ma3=(Y3-ya)/(X3-xa),//medians
	//		xc=(X1+X3)/2, yc=(Y1+Y3)/2, mc2=(Y2-yc)/(X2-xc);
	//	double
	//		px=(ma3*X3-mc2*X2+Y2-Y3)/(ma3-mc2),//centroid
	//		py=ma3*(px-X3)+Y3;

		X1-=px, Y1-=py, X2-=px, Y2-=py, X3-=px, Y3-=py;
		double t_hyp=sqrt(X1*X1+Y1*Y1), cta=X1/t_hyp, sta=Y1/t_hyp;
	//	double t_angle=atan2(Y1, X1), cta=cos(t_angle), sta=sin(t_angle);
		vv[0].x=cta*X1+sta*Y1, vv[0].y=-sta*X1+cta*Y1;
		vv[1].x=cta*X2+sta*Y2, vv[1].y=-sta*X2+cta*Y2;
		vv[2].x=cta*X3+sta*Y3, vv[2].y=-sta*X3+cta*Y3;

		r=X1*X1+Y1*Y1;				//reach
		double	dd=X2*X2+Y2*Y2;	if(r<dd)r=dd;
				dd=X3*X3+Y3*Y3;	if(r<dd)r=dd;
		r=sqrt(r);

		this->density=density;
		set_weight_triangle();
	}
	void set_properties_triangle_xy(double x1, double y1, double x2, double y2, double x3, double y3, double density)
	{
		vv.resize(3), w.resize(3);
		double
			ma3=(y3-(y1+y2)/2)/(x3-(x1+x2)/2), tX3=x3, tY3=y3,//medians
			mc2=(y2-(y1+y3)/2)/(x2-(x1+x3)/2), tX2=x2, tY2=y2;
		if(ma3==_HUGE||ma3==-_HUGE)
			ma3=(y1-(y2+y3)/2)/(x1-(x2+x3)/2), tX3=x1, tY3=y1;//b1
		else if(mc2==_HUGE||ma3==-_HUGE)
			mc2=(y1-(y2+y3)/2)/(x1-(x2+x3)/2), tX2=x1, tY2=y1;//b1
		double
			px=(ma3*tX3-mc2*tX2+tY2-tY3)/(ma3-mc2),//centroid
			py=ma3*(px-tX3)+tY3;
	//	double
	//		xa=(x1+x2)/2, ya=(y1+y2)/2, ma3=(x3-xa)/(x3-ya),
	//		xc=(x1+x3)/2, yc=(y1+y3)/2, mc2=(x2-xc)/(x2-yc);
	//	double
	//		px=(ma3*x3-mc2*x2+y2-y3)/(ma3-mc2),//centroid
	//		py=ma3*(px-x3)+y3;
		x1-=px, y1-=py, x2-=px, y2-=py, x3-=px, y3-=py;
		double hyp=sqrt(x1*x1+y1*y1);
		angle=atan2(y1, x1), ca=x1/hyp, sa=y1/hyp;
	//	angle=atan2(y1, x1), ca=cos(angle), sa=sin(angle);
		vv[0].x=ca*x1+sa*y1, vv[0].y=-sa*x1+ca*y1;
		vv[1].x=ca*x2+sa*y2, vv[1].y=-sa*x2+ca*y2;
		vv[2].x=ca*x3+sa*y3, vv[2].y=-sa*x3+ca*y3;
	//	this->x1=x1, this->y1=y1, this->x2=x2, this->y2=y2, this->x3=x3, this->y3=y3;
		this->density=density;

		r=x1*x1+y1*y1;				//reach
		double	dd=x2*x2+y2*y2;	if(r<dd)r=dd;
				dd=x3*x3+y3*y3;	if(r<dd)r=dd;
		r=sqrt(r);

		set_weight_triangle();
	}
	void set_properties_triangle_xy(double x2, double y2, double x3, double y3, double density)//relative to vertex 1
	{
		vv.resize(3), w.resize(3);
		double x1=0, y1=0;
		double
			ma3=(y3-(y1+y2)/2)/(x3-(x1+x2)/2), tX3=x3, tY3=y3,//medians
			mc2=(y2-(y1+y3)/2)/(x2-(x1+x3)/2), tX2=x2, tY2=y2;
		if(ma3==_HUGE||ma3==-_HUGE)
			ma3=(y1-(y2+y3)/2)/(x1-(x2+x3)/2), tX3=x1, tY3=y1;//b1
		else if(mc2==_HUGE||ma3==-_HUGE)
			mc2=(y1-(y2+y3)/2)/(x1-(x2+x3)/2), tX2=x1, tY2=y1;//b1
		double
			px=(ma3*tX3-mc2*tX2+tY2-tY3)/(ma3-mc2),//centroid
			py=ma3*(px-tX3)+tY3;
	//	double
	//		xa=(x1+x2)/2, ya=(y1+y2)/2, ma3=(x3-xa)/(x3-ya),
	//		xc=(x1+x3)/2, yc=(y1+y3)/2, mc2=(x2-xc)/(x2-yc);
	//	double
	//		px=(ma3*x3-mc2*x2+y2-y3)/(ma3-mc2),//centroid
	//		py=ma3*(px-x3)+y3;
		x1-=px, y1-=py, x2-=px, y2-=py, x3-=px, y3-=py;
		angle=atan2(y1, x1);
		vv[0].x=x1, vv[0].y=y1, vv[1].x=x2, vv[1].y=y2, vv[2].x=x3, vv[2].y=y3, this->density=density;

		r=x1*x1+y1*y1;				//reach
		double	dd=x2*x2+y2*y2;	if(r<dd)r=dd;
				dd=x3*x3+y3*y3;	if(r<dd)r=dd;
		r=sqrt(r);

		set_weight_triangle();
	}
	void set_weight_triangle()
	{
		double &x1=vv[0].x, &y1=vv[0].y, &x2=vv[1].x, &y2=vv[1].y, &x3=vv[2].x, &y3=vv[2].y;
		mass=0.5*abs((x1-x3)*(y2-y1)-(x2-x1)*(y1-y3))*density, inv_mass=1/mass;
		//	( 2x1(1.1+1.2+2.2) + 3x2(2.2+2.3+3.3) + 1x3(3.3+3.1+1.1) )		//https://en.wikipedia.org/wiki/List_of_moments_of_inertia
		//	/(2x1+3x2+1x3)
		double x1x1=x1*x1, y1y1=y1*y1, x2x2=x2*x2, y2y2=y2*y2, x3x3=x3*x3, y3y3=y3*y3;
		double _2x1=x2*y1-y2*x1, _3x2=x3*y2-y3*x2, _1x3=x1*y3-y1*x3;
		J=mass/6.*( _2x1*(x1x1+y1y1 + x1*x2+y1*y2 + x2x2+y2y2)
			+_3x2*(x2x2+y2y2 + x2*x3+y2*y3 + x3x3+y3y3)
			+_1x3*(x3x3+y3y3 + x3*x1+y3*y1 + x1x1+y1y1))
			/ (_2x1+_3x2+_1x3);
		inv_J=1/J;
	}

	void set_properties_rectangle(double w, double h, double density)
	{
		vv.resize(4), this->w.resize(4);
		double w2=w/2, h2=h/2;
		vv[0].set(-w2, -h2);
		vv[1].set(w2, -h2);
		vv[2].set(w2, h2);
		vv[3].set(-w2, h2);
		angle=atan2(vv[0].y, vv[0].x);
		r=sqrt(w2*w2+h2*h2);

		this->density=density;
		set_weight_rectangle(w, h);
	}
	void set_weight_rectangle(double w, double h)
	{
		mass=density*w*h, inv_mass=1/mass;
		J=mass/12*(w*w+h*h), inv_J=1/J;
	}

	void set_properties_ball(double r, int vertices, double density)
	{
		vv.resize(vertices), w.resize(vertices);
		double da=pi2/vertices, a=0;
		for(int k=0;k<vertices;++k)
			vv[k].set(r*cos(a), r*sin(a)), a+=da;
		this->r=r;

		this->density=density;
		set_weight_ball();
	}
	void set_weight_ball()
	{
		mass=density*pi*r*r, inv_mass=1/mass;
		J=0.5*mass*r*r, inv_J=1/J;
	}

	void set_position(double px, double py, double angle){p.set(px, py), this->angle=angle;ca=cos(angle), sa=sin(angle);}
	void set_velocity(double vx, double vy, double vr){v.set(vx, vy), this->vr=vr;}
	
	void get_bounding_block(RotatedRectangle &rect)
	{
		double mag_v=v.magnitude();
		if(mag_v)
		{
			Point uv=v/mag_v;
			rect.set(p+v*timescale*0.5, Point(2*r+mag_v*timescale, 2*r), uv.x, uv.y);
		}
		else//pure rotation
			rect.set(p, Point(2*r, 2*r), 1, 0);
	//	rect.draw();//
	}
	void get_bounding_block(Point &p1, Point &p2, Point &p3, Point &p4)
	{
		double mag_v=v.magnitude();
		if(mag_v)
		{
			Point vr=v*r/mag_v, vr_n(vr.y, -vr.x);
			p1=p-vr+vr_n, p2=p-vr-vr_n, p3=p+v+vr-vr_n, p4=p+v+vr+vr_n;
		//	draw_line(p-vr, p+v+vr);//
		}
		else//pure rotation
			p1.set(p.x-r, p.y-r), p2.set(p.x-r, p.y+r), p3.set(p.x+r, p.y+r), p4.set(p.x+r, p.y-r);
	//	draw_line(p1, p2), draw_line(p2, p3), draw_line(p3, p4), draw_line(p4, p1);//
	}
	void get_AABB(Point &back, Point &front)
	{
		Point rv(((v.x>=0)-(v.x<0))*r, ((v.y>=0)-(v.y<0))*r);
		back=p-rv, front=p+v*timescale+rv;
	//	draw_rectangle(back, front);//
	}
	void get_AABB_sorted(Point &BL, Point &TR)
	{
		get_AABB(BL, TR);
		sort_coordinates(BL, TR);
	}

	void draw()
	{
		for(int kv=0, kvEnd=w.size();kv<kvEnd;++kv)
			draw_line(w[kv], w[(kv+1)%w.size()]);
	}
};
std::vector<PhysPolygon> polygons0, polygons;
bool			inclusion_triangle_point(PhysPolygon const &A, Point const &p)
//bool			inclusion_triangle_point(PhysPolygon const &A, double x, double y)
{
	double
		m1x=A.w[1].x-A.w[0].x, m1y=A.w[1].y-A.w[0].y,
		m2x=A.w[0].x-A.w[2].x, m2y=A.w[0].y-A.w[2].y;
	double
		ax=intersection_lines_nonparallel_x(m1x, m1y, A.w[0].x, A.w[0].y, m2x, m2y, p.x, p.y),
		cx=intersection_lines_nonparallel_x(m2x, m2y, A.w[0].x, A.w[0].y, m1x, m1y, p.x, p.y);
	double
		a=(ax-A.w[0].x)/(A.w[1].x-A.w[0].x),
		c=(cx-A.w[0].x)/(A.w[2].x-A.w[0].x);
	return a>=0&&c>=0&&a+c<1;
}

inline double	distance_to_line_times_seg_length(Point const &p1, Point const &p2, Point const &p)//signed perpendicular distance from p to line 1->2, multiplied by magnitude(p2-p1)
{//d = (ax+by+c)/sqrt(a2+b2) = (p-p1)x(p2-p1)/sqrt(a2+b2) = (p-p1).n
	return (p.x-p1.x)*(p2.y-p1.y)-(p.y-p1.y)*(p2.x-p1.x);
	//double a=p2.y-p1.y, b=p1.x-p2.x, c=-(a*p1.x+b*p1.y);
	//return (a*p.x+b*p.y+c)/sqrt(a*a+b*b);
}
void			intersection_lines_nonparallel(Point const &l0, Point const &l1, Point const &s0, Point const &s1, Point &intersection)
{
	Point normal(l1.y-l0.y, l0.x-l1.x), s01=s1-s0;
	intersection=s0+(normal.dot(l0-s0)/normal.dot(s01))*s01;
}
//bool			intersection_lines(Point const &l0, Point const &l1, Point const &s0, Point const &s1, Point &intersection)
//{
//	Point normal(l1.y-l0.y, l0.x-l1.x), s01=s1-s0;
//	double n_d_s01=normal.dot(s01);
//	if(n_d_s01)
//	{
//		intersection=s0+(normal.dot(l0-s0)/n_d_s01)*s01;
//		return true;
//	}
//	else//parallel
//	{
//		if(normal.dot(l0-s0))
//	}
//}
bool			intersection_line_vs_segment(Point const &l0, Point const &l1, Point const &s0, Point const &s1, Point &intersection)
{
	double d_s0=distance_to_line_times_seg_length(l0, l1, s0),
		d_s1=distance_to_line_times_seg_length(l0, l1, s1);
	if((d_s0>0)!=(d_s1>0)||d_s0==0||d_s1==0)
	{
		Point normal(l1.y-l0.y, l0.x-l1.x), s01=s1-s0;
		intersection=s0+(normal.dot(l0-s0)/normal.dot(s01))*s01;
		//intersection=s0+(s1-s0)*((s1-s0).mag_sq()/(l0-s0).dot(normal));//X
		//intersection=s0+(l0-s0).dot(normal)*normal/normal.magnitude();//X
		return true;
	}
	return false;
}
bool			intersection_segments(Point const &a1, Point const &a2, Point const &b1, Point const &b2, Point &intersection)
{
	double const &ax1=a1.x, &ay1=a1.y, &ax2=a2.x, &ay2=a2.y, &bx1=b1.x, &by1=b1.y, &bx2=b2.x, &by2=b2.y;
	double
		m1x=ax2-ax1, m1y=ay2-ay1,
		m2x=bx2-bx1, m2y=by2-by1,
		temp1=m2x*m1y, temp2=m1x*m2y, cross=temp1-temp2;
	if(cross)//not parallel
	{
		double x=(temp1*ax1 - temp2*bx1 + m2x*m1x*(by1-ay1))/cross;
		double y=m1x ? m1y/m1x*(x-ax1)+ay1
			: m2y/m2x*(x-bx1)+by1;//a is vertical
		intersection.set(x, y);

		//check if (x,y) inside segments a and b
		bool in_seg_a, in_seg_b;
		if(abs(m1x)>abs(m1y))//segment a may be horizontal
			in_seg_a=ax1<ax2?ax1<=x&&x<=ax2:ax2<=x&&x<=ax1;
		else
			in_seg_a=ay1<ay2?ay1<=y&&y<=ay2:ay2<=y&&y<=ay1;
		if(abs(m2x)>abs(m2y))//segment b may be horizontal
			in_seg_b=bx1<bx2?bx1<=x&&x<=bx2:bx2<=x&&x<=bx1;
		else
			in_seg_b=by1<by2?by1<=y&&y<=by2:by2<=y&&y<=by1;
		return in_seg_a&&in_seg_b;
	}
	//parallel
	const double *axx1, *ayy1, *axx2, *bxx1;
	double m1;
	bool can_be_vertical=abs(m1x)<abs(m1y);
	if(can_be_vertical)
		axx1=&ay1, ayy1=&ax1, axx2=&ay2, bxx1=&by1, m1=m1x/m1y;
	else
		axx1=&ax1, ayy1=&ay1, axx2=&ax2, bxx1=&bx1, m1=m1y/m1x;
	double ya=m1*(*bxx1-*axx1)+*ayy1;
	if(ya==by1)//collinear
	{
		const double *ax_i, *ax_f, *bx_i, *bx_f;
		if(ax1<ax2)
			ax_i=axx1, ax_f=axx2;
		else
			ax_i=axx2, ax_f=axx1;
		if(bx1<bx2)
			bx_i=axx1, bx_f=axx2;
		else
			bx_i=axx2, bx_f=axx1;
		if(*ax_i<=*bx_f&&*ax_f>=*bx_i)//intersect
		{
			Point const *ai, *af, *bi, *bf, *i, *f;
			if(can_be_vertical)//sort by y
			{
				if(a1.y<a2.y)
					ai=&a1, af=&a2;
				else
					ai=&a2, af=&a1;
				if(b1.y<b2.y)
					bi=&b1, bf=&b2;
				else
					bi=&b2, bf=&b1;
				i=a1.y>b1.y?&a1:&b1;//upper y
				f=a2.y<b2.y?&a2:&b2;//lower y
			}
			else//sort by x
			{
				if(a1.x<a2.x)
					ai=&a1, af=&a2;
				else
					ai=&a2, af=&a1;
				if(b1.x<b2.x)
					bi=&b1, bf=&b2;
				else
					bi=&b2, bf=&b1;
				i=a1.x>b1.x?&a1:&b1;//rightmost x
				f=a2.x<b2.x?&a2:&b2;//leftmost x
			}
			intersection=(*i+*f)*0.5;
			return true;
		}
	}
	return false;
}
bool			intersection_segments(Point const &a1, Point const &a2, Point const &b1, Point const &b2)
//bool			intersection_segments(double ax1, double ay1, double ax2, double ay2, double bx1, double by1, double bx2, double by2)
{
	double const &ax1=a1.x, &ay1=a1.y, &ax2=a2.x, &ay2=a2.y, &bx1=b1.x, &by1=b1.y, &bx2=b2.x, &by2=b2.y;
	double
		m1x=ax2-ax1, m1y=ay2-ay1,
		m2x=bx2-bx1, m2y=by2-by1,
		temp1=m2x*m1y, temp2=m1x*m2y, cross=temp1-temp2;
	if(cross)//not parallel
	{
		double x=(temp1*ax1 - temp2*bx1 + m2x*m1x*(by1-ay1))/cross;
		double y=m1x ? m1y/m1x*(x-ax1)+ay1
			: m2y/m2x*(x-bx1)+by1;//a is vertical

		//check if (x,y) inside segments a and b
		bool in_seg_a, in_seg_b;
		if(abs(m1x)>abs(m1y))//segment a may be horizontal
			in_seg_a=ax1<ax2?ax1<=x&&x<=ax2:ax2<=x&&x<=ax1;
		else
			in_seg_a=ay1<ay2?ay1<=y&&y<=ay2:ay2<=y&&y<=ay1;
		if(abs(m2x)>abs(m2y))//segment b may be horizontal
			in_seg_b=bx1<bx2?bx1<=x&&x<=bx2:bx2<=x&&x<=bx1;
		else
			in_seg_b=by1<by2?by1<=y&&y<=by2:by2<=y&&y<=by1;
		return in_seg_a&&in_seg_b;
	}
	//parallel
	const double *axx1, *ayy1, *axx2, *bxx1;
	double m1;
	if(abs(m1x)>abs(m1y))
		axx1=&ax1, ayy1=&ay1, axx2=&ax2, bxx1=&bx1, m1=m1y/m1x;
	else
		axx1=&ay1, ayy1=&ax1, axx2=&ay2, bxx1=&by1, m1=m1x/m1y;
	double ya=m1*(*bxx1-*axx1)+*ayy1;
	if(ya==by1)//collinear
	{
		const double *ax_a, *ax_b, *bx_a, *bx_b;
		if(ax1<ax2)
			ax_a=axx1, ax_b=axx2;
		else
			ax_a=axx2, ax_b=axx1;
		if(bx1<bx2)
			bx_a=axx1, bx_b=axx2;
		else
			bx_a=axx2, bx_b=axx1;
		return *ax_a<=*bx_b&&*ax_b>=*bx_a;//intersect
		//return *ax_a<*bx_b&&*ax_b>*bx_a;
	}
	return false;
}
double			collision_triangles(PhysPolygon const &A, PhysPolygon const &B, bool &As_side, int &point, int &side)
{
	double vx=B.v.x-A.v.x, vy=B.v.y-A.v.y;//velocity of B as seen from A
	double fvx[]={(A.w[0].x-B.p.x)*vy-(A.w[0].y-B.p.y)*vx, (A.w[1].x-B.p.x)*vy-(A.w[1].y-B.p.y)*vx, (A.w[2].x-B.p.x)*vy-(A.w[2].y-B.p.y)*vx},//projection on perpendicular to relative velocity
		fvy[]={(A.w[0].x-B.p.x)*vx+(A.w[0].y-B.p.y)*vy, (A.w[1].x-B.p.x)*vx+(A.w[1].y-B.p.y)*vy, (A.w[2].x-B.p.x)*vx+(A.w[2].y-B.p.y)*vy};//projection on relative velocity
	int index[]={0, 1, 2};
	for(int k=0;k<2;++k)//sort coordinates by ascending x
	{
		for(int k2=k+1;k2<3;++k2)
		{
			if(fvx[k]>fvx[k2])
			{
				double temp=fvx[k]; fvx[k]=fvx[k2], fvx[k2]=temp;
				temp=fvy[k], fvy[k]=fvy[k2], fvy[k2]=temp;
				int t2=index[k]; index[k]=index[k2], index[k2]=t2;
			}
		}
	}
	int npoints;//exclude back sides
	if(fvy[1]>fvy[0]&&fvy[1]>fvy[2])//facing one (bottom) face	/_\ 
		npoints=2, fvx[1]=fvx[2], fvy[1]=fvy[2], index[1]=index[2];//remove (far) middle point
	else if(fvx[0]==fvx[1])//'middle' point is exactly in front of first point
	{
		npoints=2;
		if(fvy[0]>fvy[1])//remove farthest
			fvx[0]=fvx[1], fvy[0]=fvy[1], index[0]=index[1];
		fvx[1]=fvx[2], fvy[1]=fvy[2], index[1]=index[2];
	}
	else if(fvx[1]==fvx[2])//'middle' point is exactly in front of last point
	{
		npoints=2;
		if(fvy[1]>fvy[2])
			fvx[1]=fvx[2], fvy[1]=fvy[2], index[1]=index[2];
	}
	else
		npoints=3;
	//if(A.idx==0&&B.idx==1)//
	//{
	//	double mv=sqrt(vx*vx+vy*vy)*0.5;
	//	draw_line(w/2+fvx[0]/mv, h/2+fvy[0]/mv, w/2+fvx[1]/mv, h/2+fvy[1]/mv);//
	//	draw_line(w/2+fvx[1]/mv, h/2+fvy[1]/mv, w/2+fvx[2]/mv, h/2+fvy[2]/mv);//
	//	draw_line(w/2+fvx[2]/mv, h/2+fvy[2]/mv, w/2+fvx[0]/mv, h/2+fvy[0]/mv);//
	//	GUIPrint(ghMemDC, w/2+fvx[0]/mv, h/2+fvy[0]/mv, 0);
	//	GUIPrint(ghMemDC, w/2+fvx[1]/mv, h/2+fvy[1]/mv, 1);
	//	GUIPrint(ghMemDC, w/2+fvx[2]/mv, h/2+fvy[2]/mv, 2);
	//	draw_line(w/2, h/2, w/2+100, h/2);
	//	draw_line(w/2, h/2, w/2, h/2+100);
	//	GUIPrint(ghMemDC, w/2+(fvx[0]+fvx[1]+fvx[2])/(mv*3), h/2+(fvy[0]+fvy[1]+fvy[2])/(mv*3), "%d %d", A.idx, B.idx);
	//	//draw_line(w/2, h/2, w/2+vx, h/2);
	//	//draw_line(w/2, h/2, w/2, h/2+vy);
	//}

	double maxtravel=-1;
	for(int k=0;k<3;++k)//find which B's vertex penetrates A the most
	{
		auto &Bwx=B.w[k].x, &Bwy=B.w[k].y;
		double pvx=(Bwx-B.p.x)*vy-(Bwy-B.p.y)*vx, pvy=(Bwx-B.p.x)*vx+(Bwy-B.p.y)*vy;//B's vertex in velocity coordinates
		int face;//find which A's face B's vertex can collide
			 if(pvx<fvx[0])				continue;
		else if(pvx<fvx[1])				face=0;
		else if(npoints==3&&pvx<fvx[2])	face=1;
		else							continue;
		double travel=pvy-((fvy[face+1]-fvy[face])/(fvx[face+1]-fvx[face])*(pvx-fvx[face])+fvy[face]);
		//double travel;
		//if(face==0)
		//	travel=pvy-((fvy[1]-fvy[0])/(fvx[1]-fvx[0])*(pvx-fvx[0])+fvy[0]);
		//else
		//	travel=pvy-((fvy[2]-fvy[1])/(fvx[2]-fvx[1])*(pvx-fvx[1])+fvy[1]);
#if 0
		if(travel>50*sqrt(vx*vx+vy*vy))//
		{
			double _magv=inv_sqrt(vx*vx+vy*vy);
		//	double magv=sqrt(vx*vx+vy*vy);
			KillTimer(ghWnd, 0);
			timer=false, pause=true;
			GUIPrint(ghMemDC, 0, 0, "A: %lf, %lf", A.wx1, A.wy1);
			GUIPrint(ghMemDC, 0, 18, "%lf, %lf", A.wx2, A.wy2);
			GUIPrint(ghMemDC, 0, 18*2, "%lf, %lf", A.wx3, A.wy3);
			GUIPrint(ghMemDC, 0, 18*3, "B: %lf, %lf", B.wx1, B.wy1);
			GUIPrint(ghMemDC, 0, 18*4, "%lf, %lf", B.wx2, B.wy2);
			GUIPrint(ghMemDC, 0, 18*5, "%lf, %lf", B.wx3, B.wy3);
			GUIPrint(ghMemDC, 0, 18*6, "v: %lf, %lf, t=%lf", vx, vy, travel*_magv);
			GUIPrint(ghMemDC, 0, 18*7, "face: %d, %lf, %lf", index[0], fvx[0], fvy[0]);
			GUIPrint(ghMemDC, 0, 18*8, "%d, %lf, %lf", index[1], fvx[1], fvy[1]);
			if(npoints==3)
				GUIPrint(ghMemDC, 0, 18*9, "%d, %lf, %lf", index[2], fvx[2], fvy[2]);

			std::stringstream str;
		//	std::string str;
			str<<"A:\r\n"
				<<A.wx1<<", "<<A.wy1<<"\r\n"
				<<A.wx2<<", "<<A.wy2<<"\r\n"
				<<A.wx3<<", "<<A.wy3<<"\r\n"
				<<"B: \n"
				<<B.wx1<<", "<<B.wy1<<"\r\n"
				<<B.wx2<<", "<<B.wy2<<"\r\n"
				<<B.wx3<<", "<<B.wy3<<"\r\n"
				<<"B.v-A.v: "<<vx<<' '<<vy<<"\tm="<<1/_magv<<"\tA.vr="<<A.vr<<"\tB.vr="<<B.vr<<"\tt="<<travel*_magv<<"\r\n"
				<<"A face: npoints="<<npoints<<"\r\n"
				<<index[0]<<", "<<fvx[0]<<", "<<fvy[0]<<"\r\n"
				<<index[1]<<", "<<fvx[1]<<", "<<fvy[1]<<"\r\n";
			if(npoints==3)
				str<<index[2]<<", "<<fvx[2]<<", "<<fvy[2]<<"\r\n";
			str<<"B\r\n"
				<<k<<", "<<pvx*_magv<<", "<<pvy*_magv<<"\tt="<<travel<<'\t'<<travel*_magv;

			std::string str2=str.str();
			char *clipboard=(char*)LocalAlloc(LMEM_FIXED, (str2.size()+1)*sizeof(char));
			memcpy(clipboard, str2.c_str(), (str2.size()+1)*sizeof(char));
		/*	for(unsigned k=0;k<str2.size();k++)
				clipboard[k]=str2[k];
			clipboard[str2.size()]='\0';//*/
			OpenClipboard(ghWnd), EmptyClipboard(), SetClipboardData(CF_OEMTEXT, (void*)clipboard), CloseClipboard();
			return -1;
		}
		//	int LOL_1=0;//
#endif
		if(travel<0)
			continue;
		if(maxtravel<travel)
		{
			maxtravel=travel;
			As_side=true;
			point=k;
			side=index[face];
		}
	}
	return maxtravel;
}
double			collision_triangles_rotational(PhysPolygon const &A, PhysPolygon const &B, bool *BinsideA, bool &As_side, int &point, int &side)
{
	double maxtravel=-1;
	double vr=B.vr;//B kicks A
//	double vr=B.vr-A.vr;//B kicks A
	double vr_sign=(vr>=0)-(vr<0);
	for(int ka=0;ka<3;++ka)//each A's side
	{
		int ka_1=(ka+1)%3;
		auto &_Awx1=A.w[ka].x, &_Awx2=A.w[ka_1].x,
			&_Awy1=A.w[ka].y, &_Awy2=A.w[ka_1].y;
		
		double adxx=_Awx2-_Awx1, adx, ady;
		bool transpose=!adxx;
		double Awx1, Awy1, Awx2, Awy2;
		if(transpose)
			Awx1=_Awy1, Awy1=_Awx1, Awx2=_Awy2, Awy2=_Awx2, adx=_Awy2-_Awy1, ady=adxx;
		else
			Awx1=_Awx1, Awy1=_Awy1, Awx2=_Awx2, Awy2=_Awy2, adx=adxx, ady=_Awy2-_Awy1;
		double adx2=adx*adx, ady2=ady*ady;
		for(int kb=0;kb<3;++kb)//each B's vertex
		{
			if(BinsideA[kb])
			{
				double Bwx1, Bwy1, Bpx, Bpy;
				if(transpose)
					Bwx1=B.w[kb].y, Bwy1=B.w[kb].x, Bpx=B.p.y, Bpy=B.p.x;
				else
					Bwx1=B.w[kb].x, Bwy1=B.w[kb].y, Bpx=B.p.x, Bpy=B.p.y;
				double rx=Bwx1-Bpx, ry=Bwy1-Bpy;
				double r2=rx*rx+ry*ry;

				double
					a = adx2+ady2,
					b_2 = (Awy1-Bpy)*adx*ady-Awx1*ady2-Bpx*adx2,
				//	b_2 = ((Awy1-Bpy)*adx-Awx1*ady)*ady-Bpx*adx2
					temp=(Awy1-Bpy)*adx-Awx1*ady,
					c = (Bpx*Bpx-r2)*adx2 + temp*temp,

				//	a = adx2+ady2,
				//	b_2 = (ady-adx*(Bpx+Awx1)),
				//	c = Awx1*Awx1*ady2 - 2*Awx1*Awy1*adx*ady + adx2*(Bpx*Bpx+Awy1*Awy1-r2),

					disc_4=b_2*b_2-a*c;
				if(disc_4>0)//line and chord intersect
				{
					disc_4=1/inv_sqrt(disc_4);
					double
						x1=(-b_2+disc_4)/a,
						x2=(-b_2-disc_4)/a;
					
					double r=1/inv_sqrt(r2);
					if(Awx1<Awx2?Awx1<x1&&x1<Awx2:Awx2<x1&&x1<Awx1)
					{
						double y1=ady/adx*(x1-Awx1)+Awy1;
						//	double th2=atan2(y1-Bpy, x1-Bpx)*180/pi, th1=atan2(ry, rx)*180/pi;//
						double travel=vr_sign*(atan2(ry, rx)-atan2(y1-Bpy, x1-Bpx));//radians in velocity direction
					//	double travel=vr_sign*(atan2(ry, rx)-atan2(y1-Bpy, x1-Bpx))*r;//p1->Bw1 about B.p chord length	angle*r
						if(travel>=0&&maxtravel<travel)
							maxtravel=travel, As_side=true, point=kb, side=ka;
					}
					if(Awx1<Awx2?Awx1<x2&&x2<Awx2:Awx2<x2&&x2<Awx1)
					{
						double y2=ady/adx*(x2-Awx1)+Awy1;
						//	double th2=atan2(y2-Bpy, x2-Bpx)*180/pi, th1=atan2(ry, rx)*180/pi;//
						double travel=vr_sign*(atan2(ry, rx)-atan2(y2-Bpy, x2-Bpx));
						if(travel>=0&&maxtravel<travel)
							maxtravel=travel, As_side=true, point=kb, side=ka;
					}
				}
			}
		}
	}
	return maxtravel;
}
bool			intersection_triangles(PhysPolygon const &A, PhysPolygon const &B, bool &As_side, int &point, int &side, double &maxtravel, bool &rotational)
{
	bool
		aa=intersection_segments(A.w[0], A.w[1], B.w[0], B.w[1]),
		ab=intersection_segments(A.w[0], A.w[1], B.w[1], B.w[2]),
		ac=intersection_segments(A.w[0], A.w[1], B.w[2], B.w[0]),

		ba=intersection_segments(A.w[1], A.w[2], B.w[0], B.w[1]),
		bb=intersection_segments(A.w[1], A.w[2], B.w[1], B.w[2]),
		bc=intersection_segments(A.w[1], A.w[2], B.w[2], B.w[0]),

		b[]=
		{
			inclusion_triangle_point(A, B.w[0]),
			inclusion_triangle_point(A, B.w[1]),
			inclusion_triangle_point(A, B.w[2])
		};
	if(aa|ab|ac|ba|bb|bc|b[0]|b[1]|b[2]||
		inclusion_triangle_point(B, A.w[0]))//A inside B
	{
		double px=B.p.x-A.p.x, py=B.p.y-A.p.y,
			vx=B.v.x-A.v.x, vy=B.v.y-A.v.y, vr=B.vr-A.vr;
		double pv=px*vx+py*vy;//relative position dot relative velocity
		bool Bs_side; int Bcase_point, Bcase_side;	double travel;
		if(rotational=pv>=0)//rotational hit	B moving away from or standing still relative to A
		{
			if(rotational_pause)
			{
				KillTimer(ghWnd, 0);
				timer=false, pause=true;
			}//*/
		//	if(info)
		//		GUIPrint(ghMemDC, w/2, h/2, "rotational hit");
			bool a[]=
			{
				inclusion_triangle_point(B, A.w[0]),
				inclusion_triangle_point(B, A.w[1]),
				inclusion_triangle_point(B, A.w[2])
			};
			maxtravel	=collision_triangles_rotational(A, B, b, As_side, point, side);
			travel		=collision_triangles_rotational(B, A, a, Bs_side, Bcase_point, Bcase_side);
		}
		else//*/
		{
			maxtravel	=collision_triangles(A, B, As_side, point, side);
			travel		=collision_triangles(B, A, Bs_side, Bcase_point, Bcase_side);
		}
		if(maxtravel<travel)
			maxtravel=travel, As_side=!Bs_side, point=Bcase_point, side=Bcase_side;
		if(info&&rotational)
			GUIPrint(ghMemDC, w/2, h/2, "RH As_side=%d, point=%d, side=%d, t=%lf", As_side, point, side, maxtravel);
		return maxtravel>=0;
	}
	return false;
}

inline bool		range_y(Point const &p1, Point const &p2, Point const *&i, Point const *&f)
{
	if(p1.x==p2.x)
	{
		if(p1.y<p2.y)
			i=&p1, f=&p2;
		else
			i=&p2, f=&p1;
		return true;
	}
	if(p1.x<p2.x)
		i=&p1, f=&p2;
	else
		i=&p2, f=&p1;
	return false;
}
bool			inclusion_segment_point(Point const &p1, Point const &p2, Point const &p)
{
	double a1, a2, a, b1, b2;
	if(p1.y==p2.y)
		a1=p1.x, a2=p2.x, a=p.x;
	else
		a1=p1.y, a2=p2.y, a=p.y;
	if(a1<a2)
		b1=a1, b2=a2;
	else
		b1=a2, b2=a1;
	double d=distance_to_line_times_seg_length(p1, p2, p);
	return abs(d)<1e-10&&b1<=a&&a<=b2;
}
bool			inclusion_polygon_point(PhysPolygon const &A, Point const &p)
{
	//find minimum x coordinate
	double min_x=A.w[0].x;
	for(int k=1, kEnd=A.w.size();k<kEnd;++k)
		if(min_x>A.w[k].x)
			min_x=A.w[k].x;

	//count intersections of all A's segments with segment (min_x-10, p.y) - p
	Point p0(min_x-10, p.y);
	int n_intersections=0;
	bool on_edge=false;
	for(int k=0, kEnd=A.w.size();k<kEnd;++k)
	{
		int k2=(k+1)%A.w.size();
		n_intersections+=intersection_segments(A.w[k], A.w[k2], p0, p);
		on_edge|=inclusion_segment_point(A.w[k], A.w[k2], p);
	}
	return n_intersections&1||on_edge;
}
double			collision_polygons_convex(PhysPolygon const &A, PhysPolygon const &B, bool &As_side, int &point, int &side)
{
	double vx=B.v.x-A.v.x, vy=B.v.y-A.v.y, magv=sqrt(vx*vx+vy*vy);//velocity of B as seen from A
	vx/=magv, vy/=magv;
	std::vector<Point> rvframe(A.w.size());//relative velocity frame: x: normal to relative velocity, y: projection on relative velocity
	for(int k=0, kEnd=A.w.size();k<kEnd;++k)
		rvframe[k].set((A.w[k].x-B.p.x)*vy-(A.w[k].y-B.p.y)*vx, (A.w[k].x-B.p.x)*vx+(A.w[k].y-B.p.y)*vy);

	std::vector<int> index(A.w.size());
	for(int k=0, kEnd=A.w.size();k<kEnd;++k)
		index[k]=k;
	for(int k=0, kEnd=A.w.size()-1;k<kEnd;++k)//sort coordinates by ascending x
	{
		for(int k2=k+1, k2End=A.w.size();k2<k2End;++k2)
		{
			if(rvframe[k].x>rvframe[k2].x)
			{
				Point temp=rvframe[k]; rvframe[k]=rvframe[k2], rvframe[k2]=temp;
				int t2=index[k]; index[k]=index[k2], index[k2]=t2;
			}
		}
	}
	std::vector<int> fv;//front vertex indices, exclude back vertices
	fv.push_back(0);
	for(int k=1, kEnd=rvframe.size()-1;k<kEnd;++k)
		if(rvframe[k].y<interpolation_line_nonvertical_y(rvframe[0], *rvframe.rbegin(), rvframe[k].x))
			fv.push_back(k);
	fv.push_back(rvframe.size()-1);
	double maxtravel=-1;
	for(int k=0, kEnd=B.w.size();k<kEnd;++k)//find which B's vertex penetrates A the most
	{
		auto &v=B.w[k];
		Point pv((v.x-B.p.x)*vy-(v.y-B.p.y)*vx, (v.x-B.p.x)*vx+(v.y-B.p.y)*vy);//B's vertex in velocity coordinates
		if(pv.x<rvframe[fv[0]].x)
			continue;
		int face=-1;
		for(int k2=0, k2End=fv.size()-1;k2<k2End;++k2)
			if(rvframe[fv[k2]].x<pv.x&&pv.x<rvframe[fv[k2+1]].x)
			{
				face=k2;
				break;
			}
		if(face==-1)
			continue;
		double travel=pv.y-interpolation_line_nonvertical_y(rvframe[fv[face]], rvframe[fv[face+1]], pv.x);
	//	double travel=pvy-((rvframe[face+1].y-rvframe[face].y)/(fvx[face+1]-fvx[face])*(pvx-fvx[face])+fvy[face]);
		if(travel<0)
			continue;
		if(maxtravel<travel)
		{
			maxtravel=travel;
			As_side=true;
			point=k;
			side=index[fv[face]];
		}
	}
	return maxtravel;
}
double			collision_polygons_convex_rotational(PhysPolygon const &A, PhysPolygon const &B, std::vector<char> const &BinsideA, bool &As_side, int &point, int &side)
{
	double maxtravel=-1;
	double vr=B.vr;//assume B kicks A
	double vr_sign=(vr>=0)-(vr<0);
	for(int ka=0, kaEnd=A.w.size();ka<kaEnd;++ka)//each A's side
	{
		int ka1=(ka+1)%A.w.size();
		auto &_Awx1=A.w[ka].x, &_Awx2=A.w[ka1].x,
			&_Awy1=A.w[ka].y, &_Awy2=A.w[ka1].y;
		
		double adxx=_Awx2-_Awx1, adx, ady;
		bool transpose=!adxx;
		double Awx1, Awy1, Awx2, Awy2;
		if(transpose)
			Awx1=_Awy1, Awy1=_Awx1, Awx2=_Awy2, Awy2=_Awx2, adx=_Awy2-_Awy1, ady=adxx;
		else
			Awx1=_Awx1, Awy1=_Awy1, Awx2=_Awx2, Awy2=_Awy2, adx=adxx, ady=_Awy2-_Awy1;
		double adx2=adx*adx, ady2=ady*ady;
		for(int kb=0, kbEnd=B.w.size();kb<kbEnd;++kb)//each B's vertex
		{
			if(BinsideA[kb])
			{
				double Bwx1, Bwy1, Bpx, Bpy;
				if(transpose)
					Bwx1=B.w[kb].y, Bwy1=B.w[kb].x, Bpx=B.p.y, Bpy=B.p.x;
				else
					Bwx1=B.w[kb].x, Bwy1=B.w[kb].y, Bpx=B.p.x, Bpy=B.p.y;
				double rx=Bwx1-Bpx, ry=Bwy1-Bpy;//centroid -> vertex k
				double r2=rx*rx+ry*ry;

				double
					a = adx2+ady2,
					b_2 = (Awy1-Bpy)*adx*ady-Awx1*ady2-Bpx*adx2,
				//	b_2 = ((Awy1-Bpy)*adx-Awx1*ady)*ady-Bpx*adx2
					temp=(Awy1-Bpy)*adx-Awx1*ady,
					c = (Bpx*Bpx-r2)*adx2 + temp*temp,

				//	a = adx2+ady2,
				//	b_2 = (ady-adx*(Bpx+Awx1)),
				//	c = Awx1*Awx1*ady2 - 2*Awx1*Awy1*adx*ady + adx2*(Bpx*Bpx+Awy1*Awy1-r2),

					disc_4=b_2*b_2-a*c;
				if(disc_4>0)//line and chord intersect
				{
					disc_4=1/inv_sqrt(disc_4);
					double
						x1=(-b_2+disc_4)/a,
						x2=(-b_2-disc_4)/a;
					
					double r=1/inv_sqrt(r2);
					if(Awx1<Awx2?Awx1<x1&&x1<Awx2:Awx2<x1&&x1<Awx1)
					{
						double y1=ady/adx*(x1-Awx1)+Awy1;
						//	double th2=atan2(y1-Bpy, x1-Bpx)*180/pi, th1=atan2(ry, rx)*180/pi;//
						double travel=vr_sign*(atan2(ry, rx)-atan2(y1-Bpy, x1-Bpx));//radians in velocity direction
					//	double travel=vr_sign*(atan2(ry, rx)-atan2(y1-Bpy, x1-Bpx))*r;//p1->Bw1 about B.p chord length	angle*r
						if(travel>=0&&maxtravel<travel)
							maxtravel=travel, As_side=true, point=kb, side=ka;
					}
					if(Awx1<Awx2?Awx1<x2&&x2<Awx2:Awx2<x2&&x2<Awx1)
					{
						double y2=ady/adx*(x2-Awx1)+Awy1;
						//	double th2=atan2(y2-Bpy, x2-Bpx)*180/pi, th1=atan2(ry, rx)*180/pi;//
						double travel=vr_sign*(atan2(ry, rx)-atan2(y2-Bpy, x2-Bpx));
						if(travel>=0&&maxtravel<travel)
							maxtravel=travel, As_side=true, point=kb, side=ka;
					}
				}
			}
		}
	}
	return maxtravel;
}
bool			intersection_polygons(PhysPolygon const &A, PhysPolygon const &B, bool &As_side, int &point, int &side, double &maxtravel, bool &rotational)
{
	bool intersection=false;
	for(int ka=0, kaEnd=A.w.size()-1;ka<kaEnd&&!intersection;++ka)
		for(int kb=0, kbEnd=B.w.size();kb<kbEnd&&!intersection;++kb)
			intersection|=intersection_segments(A.w[ka], A.w[ka+1], B.w[kb], B.w[(kb+1)%B.w.size()]);
	std::vector<char> b(B.w.size());
	for(int kb=0, kbEnd=B.w.size();kb<kbEnd;++kb)//B's vertex inclusions
		intersection|=b[kb]=inclusion_polygon_point(A, B.w[kb]);
	if(intersection||inclusion_polygon_point(B, A.w[0]))//A intersects with B or A inside B
	{
		//int *hguard=new int;//
		double px=B.p.x-A.p.x, py=B.p.y-A.p.y,
			vx=B.v.x-A.v.x, vy=B.v.y-A.v.y, vr=B.vr-A.vr;
		double pv=px*vx+py*vy;//relative position dot relative velocity
		bool Bs_side;
		int Bcase_point, Bcase_side;
		double travel;
	//	rotational=false;//
		if(rotational=pv>=0)//rotational hit	B moving away from or standing still relative to A
		{
			std::vector<char> a(A.w.size());
			for(int ka=0, kaEnd=A.w.size();ka<kaEnd;++ka)//A's vertex inclusions
				intersection|=a[ka]=inclusion_polygon_point(B, A.w[ka]);
			maxtravel	=collision_polygons_convex_rotational(A, B, b, As_side, point, side);
			travel		=collision_polygons_convex_rotational(B, A, a, Bs_side, Bcase_point, Bcase_side);
		}
		else
		{
			maxtravel	=collision_polygons_convex(A, B, As_side, point, side);
			travel		=collision_polygons_convex(B, A, Bs_side, Bcase_point, Bcase_side);
		}
		if(maxtravel<travel)
			maxtravel=travel, As_side=!Bs_side, point=Bcase_point, side=Bcase_side;
		if(info&&rotational)
			GUIPrint(ghMemDC, w/2, h/2, "RH As_side=%d, point=%d, side=%d, t=%lf", As_side, point, side, maxtravel);
		//delete hguard;//
		return maxtravel>=0;
	}
	return false;
}

struct			PolytopeVertex
{
	Point p;
	int a_idx, b_idx;
	PolytopeVertex():a_idx(-1), b_idx(-1){}
	PolytopeVertex(Point const &p, int a_idx, int b_idx):p(p), a_idx(a_idx), b_idx(b_idx){}
	void set(Point const &p, int a_idx, int b_idx){this->p=p, this->a_idx=a_idx, this->b_idx=b_idx;}
};
struct			Simplex
{
	PolytopeVertex a, b, c;
	int count;
	Simplex():count(0){}
	Simplex(PolytopeVertex const &a):count(1), a(a){}
	Simplex(PolytopeVertex const &a, PolytopeVertex const &b):count(2), a(a), b(b){}
	Simplex(Point const &a, int a_idx, int b_idx):count(1), a(a, a_idx, b_idx){}
	//Simplex(Point const &a, int a_idx, int b_idx, Point const &b, int a_idx2, int b_idx2):count(2), a(a, a_idx, b_idx), b(b, a_idx2, b_idx2){}
	//Simplex(Point const &a, Point const &b, Point const &c):count(3), a(a), b(b), c(c){}
	void add(Point const &p, int a_idx, int b_idx)
	{
		if(count<3)
			(&a)[count].set(p, a_idx, b_idx), ++count;
	}
};
Point			support(PhysPolygon const &S, Point const &d, int &index)
{
	index=0;
	double max_dot=S.w[0].dot(d);
	for(int k=1, kEnd=S.w.size();k<kEnd;++k)
	{
		double dot=S.w[k].dot(d);
		if(max_dot<dot)
			max_dot=dot, index=k;
	}
	return S.w[index];
}
/*Point			support(PhysPolygon const &S, Point const &d)
{
	int point=0;
	double max_dot=S.w[0].dot(d);
	for(int k=1, kEnd=S.w.size();k<kEnd;++k)
	{
		double dot=S.w[k].dot(d);
		if(max_dot<dot)
			max_dot=dot, point=k;
	}
	return S.w[point];
}//*/
void			get_uv_line(Point const &a, Point const &b, double &u, double &v, Point &n)
{
	n=b-a;
	n/=n.mag_sq();
	//if(n.x>0&&n.y>0)//should point towards origin
	//	n=-n;
	v=-a.dot(n), u=b.dot(n);
}
void			set_GJK_direction(Point const &n, Point const &p, Point &d)
{
	d.set(n.y, -n.x);
	if(d.dot(p)>0)
		d=-d;
}
bool			intersection_polygons_GJK(PhysPolygon const &P, PhysPolygon const &Q, Point const &initial_axis, Simplex &s)//initial axis: any starting direction works (relative velocity?), returns closest simplex to origin
{
	//if(P.idx==1&&Q.idx==8)
	//	int LOL_1=0;
	int p_idx, q_idx;
	Point a=support(P, initial_axis, p_idx)-support(Q, -initial_axis, q_idx);
	s=Simplex(a, p_idx, q_idx);
	//Simplex s(a);
	Point d=-a;
	bool contains_origin=false;
	for(;;)
	{
		a=support(P, d, p_idx)-support(Q, -d, q_idx);
		if(a.dot(d)<0)
			return false;
		s.add(a, p_idx, q_idx);
		//nearest simplex (s) -> s,d,contains_origin
		switch(s.count)
		{
		case 1:
			contains_origin=!s.a.p.mag_sq();
			break;
		case 2://line segment: find nearest point or line to the origin
			{
				Point n;
				double u, v;
				get_uv_line(s.a.p, s.b.p, u, v, n);
				Point p;
				if(u<=0)
					p=s.b.p, s=Simplex(s.b), d=-p;
			//	else if(v<=0)//should never hit (a is old)
			//		p=s.a.p, s=Simplex(s.a), d=-p;
				else
				{
					p=u*s.a.p+v*s.b.p;
					set_GJK_direction(n, p, d);
					//d.set(n.y, -n.x);
					//if(d.dot(p)>0)
					//	d=-d;
				}
				contains_origin=!p.mag_sq();
			}
			break;
		case 3://triangle
			{
				Point n, nBC, nCA;
				double uBC, vBC, uCA, vCA;
			//	Point n, nAB, nBC, nCA;
			//	double uAB, vAB, uBC, vBC, uCA, vCA;
			//	get_uv_line(s.a.p, s.b.p, uAB, vAB, nAB);//ab is old
				get_uv_line(s.b.p, s.c.p, uBC, vBC, nBC);
				get_uv_line(s.c.p, s.a.p, uCA, vCA, nCA);
				Point p;
			//	if(vAB<=0&&uCA<=0)//a is old
			//		p=s.a.p, s=Simplex(s.a), d=-p;
			//	else if(uAB<=0&&vBC<=0)//b is old
			//		p=s.b.p, s=Simplex(s.b), d=-p;
				if(uBC<=0&&vCA<=0)
					p=s.c.p, s=Simplex(s.c), d=-p;
				else
				{
					double area=(s.b.p-s.a.p).cross(s.c.p-s.a.p),//ABC
						uABC=s.b.p.cross(s.c.p)/area,//OBC/ABC
						vABC=s.c.p.cross(s.a.p)/area;//OCA/ABC
					//	wABC=s.a.p.cross(s.b.p)/area;//OAB/ABC
				//	if(uAB>0&&vAB>0&&wABC<=0)//should never hit, ab is old
				//		p=uAB*s.a.p+vAB*s.b.p, s=Simplex(s.a, s.b), set_GJK_direction(nAB, p, d);
					if(uBC>0&&vBC>0&&uABC<=0)
						p=uBC*s.b.p+vBC*s.c.p, s=Simplex(s.b, s.c), set_GJK_direction(nBC, p, d);
					else if(uCA>0&&vCA>0&&vABC<=0)
						p=uCA*s.b.p+vCA*s.c.p, s=Simplex(s.c, s.a), set_GJK_direction(nCA, p, d);
					else//the origin is inside ABC, return
						return true;
					//	p.set(0, 0);
					//	p.set(0, 0), d.set(0, 0);
					//	p=uABC*s.a.p+vABC*s.b.p+wABC*s.c.p, n.set(0, 0);
					//d.set(n.y, -n.x);
					//if(d.dot(p)>0)
					//	d=-d;
				}
				contains_origin=!p.mag_sq();
			}
			break;
		}
		if(contains_origin)
			break;
	}
	return contains_origin;
}

Point			support_website(PhysPolygon const &P, PhysPolygon const &Q, Point const &d, int &p_idx, int &q_idx)
{
	return support(P, d, p_idx)-support(Q, -d, q_idx);
}
Point			triple_product(Point const &a, Point const &b, Point const &c)
{//(a x b) x c = b(c.a) - a(b.c)
	return b*(c.dot(a))-a*(b.dot(c));
}
bool			contains_origin_website(Simplex &s, Point &d)
{
	if(s.count==3)//triangle
	{
		PolytopeVertex &a=s.c,//latest
			&b=s.b,
			&c=s.a;//first
		Point ao=-a.p;
		Point ab=b.p-a.p,
			ac=c.p-a.p;
		Point
			abPerp=triple_product(ac, ab, ab),
			acPerp=triple_product(ab, ac, ac);
		if(abPerp.dot(ao)>0)
		{
			s=Simplex(b, a);//remove c
			d=abPerp;
		}
		else if(acPerp.dot(ao))
		{
			s=Simplex(c, a);//remove b
			d=acPerp;
		}
		else
			return true;
	}
	else//line segment
	{
		Point &a=s.b.p,//latest
			&b=s.a.p;//first
		Point ao=-a;
		Point ab=b-a;//latest->first
		Point abPerp=triple_product(ab, ao, ab);
		d=abPerp;
	}
	return false;
}
bool			intersection_polygons_GJK_website(PhysPolygon const &P, PhysPolygon const &Q, Point const &initial_axis, Simplex &s)
{//http://www.dyn4j.org/2010/04/gjk-gilbert-johnson-keerthi/
	if(P.idx==1&&Q.idx==8)
		int LOL_1=0;
	Point d=initial_axis;
	int p_idx, q_idx;
	Point point=support_website(P, Q, d, p_idx, q_idx);
	s.add(point, p_idx, q_idx);
	d=-d;
	for(;;)
	{
		point=support_website(P, Q, d, p_idx, q_idx);
		s.add(point, p_idx, q_idx);
		if((&s.a.p)[s.count-1].dot(d)<=0)
			return false;
		if(contains_origin_website(s, d))
			return true;
	}
}

bool			intersection_EPA(PhysPolygon const &A, PhysPolygon const &B, Point &penetration_vector, double &penetration_depth, bool &not_sure, Point &ca, Point &cb)
{//https://github.com/CG-F15-9-Rutgers/SteerLite/blob/master/steerlib/src/GJK_EPA.cpp
	not_sure=false;
	Simplex s;
	if(intersection_polygons_GJK(A, B, A.p-B.p, s))
	{
		int a_idx, b_idx;
		switch(s.count)//expand the simplex to triangle
		{
		case 1://point
			{
				Point point=support(A, s.a.p, a_idx)-support(B, -s.a.p, b_idx);
				s.add(point, a_idx, b_idx);
			}//continued in line case
		case 2://line segment
			{
				Point edge=s.b.p-s.a.p, n(edge.y, -edge.x);
				if(n.dot(s.a.p)<0)
					n=-n;
				Point point=support(A, n, a_idx)-support(B, -n, b_idx);
				s.add(point, a_idx, b_idx);
			}
			break;
		}

		std::vector<PolytopeVertex> pt(3);//polytope
		pt[0]=s.a, pt[1]=s.b, pt[2]=s.c;
		//std::vector<Point> pt(s.count);
		//for(int k=0;k<s.count;++k)
		//	pt[k]=(&s.a)[k];

		int index;
		for(;;)
		{
			const double TOLERANCE=1e-10;
			double min_distance=_HUGE;
			Point normal;

			for(int k=0, kEnd=pt.size();k<kEnd;++k)//get nearest edge
			{
				int k2=(k+1)%kEnd;
				Point &v1=pt[k].p, &v2=pt[k2].p;
				Point edge=v2-v1;//for each edge in polytope
				Point n=v1*(edge.dot(edge))-edge*(edge.dot(v1));//(AxB)xC = B(C.dot(A))–A(C.dot(B))	triple product to get vector from edge to the origin
				n/=n.magnitude();
				double dist=n.dot(v1);
				if(min_distance>dist)
				{
					min_distance=dist;
					index=k, normal=n;
				}
			}
			Point sup=support(A, normal, a_idx)-support(B, -normal, b_idx);
			double d=sup.dot(normal);
			if(d-min_distance<=TOLERANCE)
			{
				penetration_vector=normal;
				penetration_depth=min_distance;
				break;
			}
			else//insert sup
			{
				bool duplicate=false;
				for(int k=0, kEnd=pt.size();k<kEnd;++k)
					if(pt[k].p==sup)
					{
						duplicate=true;
						break;
					}
				if(duplicate)//
				{
					penetration_vector=normal;
					penetration_depth=min_distance;
					not_sure=true;
					break;
				}
				pt.insert(pt.begin()+(index+1)%pt.size(), PolytopeVertex(sup, a_idx, b_idx));
			}
		}
		//find contact point
		//if(frame_number==13)
		//	int LOL_1=0;
		int index_v2=(index+1)%pt.size();
		PolytopeVertex &v1=pt[index], &v2=pt[index_v2];
		int ka1=v1.a_idx, ka2=v2.a_idx,
			kb1=v1.b_idx, kb2=v2.b_idx;
		double u, v;
		Point n, p;
		get_uv_line(v1.p, v2.p, u, v, n);
		if(u<=0)
			ca=A.w[v2.a_idx], cb=B.w[v2.b_idx];//, p=v2.p;
		else if(v<=0)
			ca=A.w[v1.a_idx], cb=B.w[v1.b_idx];//, p=v1.p;
		else
			ca=u*A.w[v1.a_idx]+v*A.w[v2.a_idx], cb=u*B.w[v1.b_idx]+v*B.w[v2.b_idx];//, p=u*v1.p+v*v2.p;
		return true;
	}
	return false;
}

void			find_least_penetration(PhysPolygon const &A, PhysPolygon const &B, double &max_dist, int &Aface_idx, int &Bv_idx, Point &normal)
{
	for(unsigned ke=0;ke<A.w.size();++ke)
	{
		Point const &a0=A.w[ke], &a1=A.w[(ke+1)%A.w.size()];
		Point an(-a1.y+a0.y, -a0.x+a1.x);//points into A
		int kv;
		Point vb=support(B, an, kv);
		double dist=distance_to_line_times_seg_length(a0, a1, vb);
		if(max_dist<dist)
			max_dist=dist, Aface_idx=ke, Bv_idx=kv, normal=-an;//points out of A
	}
}
int				manifold_support(std::vector<Point> const &out, Point const &direction)
{
	int index=0;
	double max_dot=out[0].dot(direction);
	for(int k=1, kEnd=out.size();k<kEnd;++k)
	{
		double dot=out[k].dot(direction);
		if(max_dot<dot)
			max_dot=dot, index=k;
	}
	return index;
}
//struct			Edge
//{
//	Point p0, p1;
//	Edge():p0(0, 0), p1(0, 0){}
//	Edge(Point const &p0, Point const &p1):p0(p0), p1(p1){}
//	void set(Point const &p0, Point const &p1){this->p0=p0, this->p1=p1;}
//};
//bool			intersection_SAT(PhysPolygon const &A, PhysPolygon const &B, double &depth, Point &normal, Point *ca, int &count_a, Point *cb, int &count_b)
bool			intersection_SAT(PhysPolygon const &A, PhysPolygon const &B, double &depth, Point &normal, Point *cp, int &count)
{
	double Amax_dist=-_HUGE;
	int Aface_idx, Bv_idx;
	Point an;
	find_least_penetration(A, B, Amax_dist, Aface_idx, Bv_idx, an);
	if(Amax_dist>0)
		return false;
	double Bmax_dist=-_HUGE;
	int Bface_idx, Av_idx;
	Point bn;
	find_least_penetration(B, A, Bmax_dist, Bface_idx, Av_idx, bn);
	if(Bmax_dist>0)
		return false;
//	count=0;


/*	bool swapped=Amax_dist<Bmax_dist;//A's side: largest negative penetration
	PhysPolygon const *A2, *B2;
	if(swapped)
	{
		A2=&B, B2=&A;
		Point const &b0=B.w[Bface_idx], &b1=B.w[(Bface_idx+1)%B.w.size()];
		depth=-Bmax_dist, normal.set(b0.y-b1.y, b1.x-b0.x);//pointing out of A
	//	draw_line(b0, b0+normal);
	}
	else
	{
		A2=&A, B2=&B;
		Point const &a0=A.w[Aface_idx], &a1=A.w[(Aface_idx+1)%A.w.size()];
		depth=-Amax_dist, normal.set(a1.y-a0.y, a0.x-a1.x);//pointing out of A
	//	draw_line(a0, a0+normal);
	}
	normal/=normal.magnitude();
	//Edge reference(A2->w[Aface_idx], A2->w[(Aface_idx+1)%A2->w.size()]);
	Edge incident (B2->w[Bface_idx], B2->w[(Bface_idx+1)%B2->w.size()]);
	//Point const &a0=A2->w[Aface_idx], &a1=A2->w[(Aface_idx+1)%A2->w.size()];
	//Point const &b0=B2->w[Bface_idx], &b1=B2->w[(Bface_idx+1)%B2->w.size()];
	//reference.set(a0, a1);
	//incident.set(b0, b1);
	bool incident_exists=true;
	Edge in(incident);
//	Edge in(incident), out;
	if(frame_number==14&&A.idx==1&&B.idx==5)
		int LOL_1=0;
	for(int ka=0, a_size=A2->w.size();ka<a_size;++ka)
	{
		Edge a(A2->w[ka], A2->w[(ka+1)%a_size]);
		double d_b0=distance_to_line_times_seg_length(a.p1, a.p0, in.p0),
			d_b1=distance_to_line_times_seg_length(a.p1, a.p0, in.p1);
		if(d_b0>0)
		{
			if(d_b1>0)//both inside, save b1
				;
			else//b0 inside, b1 outside, save i
			{
				Point i;
				intersection_lines_nonparallel(a.p0, a.p1, in.p0, in.p1, i);
				in.p1=i;
			}
		}
		else if(d_b1>0)//b0 outside, b1 inside, store i & b1
		{
			Point i;
			intersection_lines_nonparallel(a.p0, a.p1, in.p0, in.p1, i);
			in.p1=i;
		}
		else//both outside, none saved
		{
			incident_exists=false;
			break;
		}
		//if(d_b0>0)
		//{
		//	if(d_b1>0)//both inside, save b1
		//	{
		//		out.p0=in.p0;
		//		out.p1=in.p1;
		//	}
		//	else//b0 inside, b1 outside, save i
		//	{
		//		Point i;
		//		intersection_lines_nonparallel(a0, a1, b0, b1, i);
		//		out.p0=in.p0;
		//		out.p1=i;
		//	}
		//}
		//else if(d_b1>0)//b0 outside, b1 inside, store i & b1
		//{
		//	Point i;
		//	intersection_lines_nonparallel(a0, a1, b0, b1, i);
		//	out.p0=i;
		//	out.p1=in.p1;
		//}
		////else both outside, none saved
		//in=out;
	}
	if(incident_exists)
	{
		count=2, cp[0]=in.p0, cp[1]=in.p1;
		draw_line(cp[0]-10, cp[0]+10), draw_line(cp[0].x-10, cp[0].y+10, cp[0].x+10, cp[0].y-10);//
		draw_line(cp[1]-10, cp[1]+10), draw_line(cp[1].x-10, cp[1].y+10, cp[1].x+10, cp[1].y-10);//
	}
	else
	{
		count=0;
		GUIPrint(ghMemDC, A.p.x, A.p.y, "no incident");
		GUIPrint(ghMemDC, B.p.x, B.p.y, "no incident");
		draw_line(incident.p0+10, incident.p1+10);
	}//*/

	bool As_side=Amax_dist>Bmax_dist;//largest negative penetration
	int reference, incident;
	if(As_side)
	{
		reference=Aface_idx, incident=Bface_idx;
		Point const &a0=A.w[Aface_idx], &a1=A.w[(Aface_idx+1)%A.w.size()];
		depth=-Amax_dist, normal.set(a1.y-a0.y, a0.x-a1.x);//pointing out of A
	//	draw_line(a0, a0+normal);
	}
	else
	{
		reference=Bface_idx, incident=Aface_idx;
		Point const &b0=B.w[Bface_idx], &b1=B.w[(Bface_idx+1)%B.w.size()];
		depth=-Bmax_dist, normal.set(b0.y-b1.y, b1.x-b0.x);//pointing out of A
	//	draw_line(b0, b0+normal);
	}
	normal/=normal.magnitude();

	std::vector<Point> in=B.w, out;//crop of B inside A - Sutherland–Hodgman algorithm
	for(int ka=0, a_size=A.w.size();ka<a_size;++ka)
	{
		Point const &a0=A.w[ka], &a1=A.w[(ka+1)%a_size];
		for(unsigned kb=0, b_size=in.size();kb<b_size;++kb)
		{
			Point const &b0=in[kb], &b1=in[(kb+1)%b_size];
			double d_b0=distance_to_line_times_seg_length(a1, a0, b0),
				d_b1=distance_to_line_times_seg_length(a1, a0, b1);
			if(d_b0>0)
			{
				if(d_b1>0)//both inside, save b1
					out.push_back(b1);
				else//b0 inside, b1 outside, save i
				{
					Point i;
					intersection_lines_nonparallel(a0, a1, b0, b1, i);
					out.push_back(i);
				}
			}
			else if(d_b1>0)//b0 outside, b1 inside, store i & b1
			{
				Point i;
				intersection_lines_nonparallel(a0, a1, b0, b1, i);
				out.push_back(i);
				out.push_back(b1);
			}
			//else both outside, none saved
		}
		in=std::move(out);
		//HPEN hPen=CreatePen(PS_DOT, 1, 0xFF0000);
		//hPen=(HPEN)SelectObject(ghMemDC, hPen);
		//for(int ki=0, i_size=in.size();ki<i_size;++ki)
		//{
		//	Point p0=2*(in[ki]-B.p)+w/2-ka*100,
		//		p1=2*(in[(ki+1)%i_size]-B.p)+w/2-ka*100;
		//	draw_line(p0, p1);
		//	GUIPrint(ghMemDC, p0.x, p0.y, ki);
		//}
		//hPen=(HPEN)SelectObject(ghMemDC, hPen);
		//DeleteObject(hPen);
	}
	//GUIPrint(ghMemDC, B.idx*75., h/2+100.+A.idx*18, "%d vs %d", A.idx, B.idx);
	//HPEN hPen=CreatePen(PS_DOT, 1, 0xFF0000);
	//hPen=(HPEN)SelectObject(ghMemDC, hPen);
	//for(int ki=0, i_size=in.size();ki<i_size;++ki)
	//{
	//	Point p0=4*(in[ki]-B.p), p1=4*(in[(ki+1)%i_size]-B.p);
	//	draw_line(p0.x+B.idx*75, p0.y+h/2, p1.x+B.idx*75, p1.y+h/2);
	//	GUIPrint(ghMemDC, p0.x+B.idx*75, p0.y+h/2, ki);
	////	Point p0=in[ki]+2, p1=in[(ki+1)%i_size]+2;
	//	//draw_line(p0, p1);
	//	//GUIPrint(ghMemDC, p0.x, p0.y, ki);
	//}
	//hPen=(HPEN)SelectObject(ghMemDC, hPen);
	//DeleteObject(hPen);

	if(in.size()==1)
		cp[0]=in[0], count=1;
	else if(in.size()>1)
	{
		Point const *a0, *a1;
		if(As_side)
			a0=&A.w[Aface_idx], a1=&A.w[(Aface_idx+1)%A.w.size()];
		else
			a0=&B.w[Bface_idx], a1=&B.w[(Bface_idx+1)%B.w.size()];
		count=2;
		cp[0]=in[manifold_support(in, *a0-*a1)];
		cp[1]=in[manifold_support(in, *a1-*a0)];
		//draw_line(cp[0]-10, cp[0]+10), draw_line(cp[0].x-10, cp[0].y+10, cp[0].x+10, cp[0].y-10);
		//draw_line(cp[1]-10, cp[1]+10), draw_line(cp[1].x-10, cp[1].y+10, cp[1].x+10, cp[1].y-10);
	}
	else count=0;

/*	if(info)
	{
		HPEN hPen=CreatePen(PS_DOT, 1, 0xFF0000);
		hPen=(HPEN)SelectObject(ghMemDC, hPen);
		GUIPrint(ghMemDC, A.p.x, A.p.y, "intersect");
		GUIPrint(ghMemDC, B.p.x, B.p.y, "intersect");
		for(int kp=0;kp<count;++kp)
			draw_line(A.p, cp[kp]);
		//draw_line(B.w[Bv_idx], B.w[Bv_idx]+an);
		//draw_line(A.w[Av_idx], A.w[Av_idx]+bn);
		hPen=(HPEN)SelectObject(ghMemDC, hPen);
		DeleteObject(hPen);
		draw_line(cp[0], cp[0]+normal*100);
	}//*/
	return true;
}

//struct			PII
//{
//	Point p;
//	int ka, kb;
//	PII(Point const &p, int ka, int kb):p(p), ka(ka), kb(kb){}
//};
//bool			intersection_BF(PhysPolygon const &A, PhysPolygon const &B, double &depth, Point &normal, Point *cp, int &count)//brute force, up to 4 points for convex polygons
//{
//	count=0;
//	std::vector<PII> points;
//	for(int ka=0, a_size=A.w.size();ka<a_size;++ka)
//	{
//		Point const &a0=A.w[ka], &a1=A.w[(ka+1)%a_size];
//		for(int kb=0, b_size=B.w.size();kb<b_size;++kb)
//		{
//			Point const &b0=B.w[kb], &b1=B.w[(kb+1)%b_size];
//			Point intersection, na, nb;
//			if(intersection_segments(a0, a1, b0, b1, intersection))
//				points.push_back(PII(intersection, ka, kb));
//			//	cp[count]=intersection, ++count;
//		}
//	}
//	for(int k=0, kEnd=points.size();k<2&&k<kEnd;++k)
//		cp[k]=points[k];
//	return points.size()>0;
//}

bool			resolve_collision_polygon_vs_wall(PhysPolygon &A, int point, double nx, double ny, double travel, bool others_side)
{
	A.translate_world(Point(nx*travel, ny*travel));

	double cx, cy;
	if(others_side)//point collision
		cx=A.w[point].x, cy=A.w[point].y;
	else//face collision
	{
		int p2=(point+1)%A.w.size();
		cx=(A.w[point].x+A.w[p2].x)*0.5, cy=(A.w[point].y+A.w[p2].y)*0.5;
	}
//	double &cx=A.w[point].x, &cy=A.w[point].y;
	double rax=cx-A.p.x, ray=cy-A.p.y, sa=rax*ny-ray*nx;
	double lambda=damping*2*( (A.v.x)*nx+(A.v.y)*ny + A.vr*sa )
		/ ( (nx*nx+ny*ny)*A.inv_mass + sa*sa*A.inv_J );
	if(info)
	{
		GUIPrint(ghMemDC, cx+5, cy-5, "%lf", lambda);
		draw_rectangle(cx-5, cx+5, cy-5, cy+5);
	}
	if(abs(lambda)<A.mass*10)//200
		return true;
	double lambda_ma=lambda*A.inv_mass;
	A.v.x-=lambda_ma*nx, A.v.y-=lambda_ma*ny, A.vr-=lambda*sa*A.inv_J;//*/
	
	lastlambda_walls=lambda;//
	return false;
}
bool			resolve_collision_polygon_vs_polygon(PhysPolygon &A, PhysPolygon &B, bool As_side, int point, int side, double travel, bool rotational)
{
	//if((A.idx==5||A.idx==8)&&(B.idx==5||B.idx==8))
	//	int LOL_1=0;
	PhysPolygon *A2, *B2;
	bool swapped=!As_side;
	if(swapped)
		A2=&B, B2=&A;
	else
		A2=&A, B2=&B;
	
	double &cx=B2->w[point].x, &cy=B2->w[point].y;
	int side_v2=(side+1)%A2->w.size();
	double
		&lx1=A2->w[side].x, &lx2=A2->w[side_v2].x,//collision line, far point, collision point
		&ly1=A2->w[side].y, &ly2=A2->w[side_v2].y;
					
	double nx=ly2-ly1, ny=-(lx2-lx1);//collision surface normal (pointing out of A2)
	if(rotational)
	{
		if(swapped)
			B2->angle-=travel, B2->udpate_angle(), B2->update_world_coord();
		else
			A2->angle+=travel, A2->udpate_angle(), A2->update_world_coord();
	//	B2->angle-=travel*0.1, B2->udpate_angle(), B2->update_world_coord();
	}
	else
	{
	//	double dvx=A2->vx-B2->vx, dvy=A2->vy-B2->vy, magdvsq=dvx*dvx+dvy*dvy;
		double dvx=A2->v.x-B2->v.x, dvy=A2->v.y-B2->v.y, magdv=sqrt(dvx*dvx+dvy*dvy);
		if(magdv)
		{
			//dvx/=magdvsq, dvy/=magdvsq;
			//A2->px-=travel*dvx/2, A2->py-=travel*dvy/2, A2->update_world_coord();
			//B2->px+=travel*dvx/2, B2->py+=travel*dvy/2, B2->update_world_coord();
			dvx/=magdv, dvy/=magdv;
			//dvx/=magdv*2*10, dvy/=magdv*2*10;//0.1 of travel
			if(swapped)
				B2->translate_world(Point(travel*dvx, travel*dvy));
			//	B2->p.x+=travel*dvx, B2->p.y+=travel*dvy, B2->update_world_coord();
			else
				A2->translate_world(Point(-travel*dvx, -travel*dvy));
			//	A2->p.x-=travel*dvx, A2->p.y-=travel*dvy, A2->update_world_coord();
		}
		else
		{
			double mag_n=sqrt(nx*nx+ny*ny);
			dvx=nx/mag_n*2, dvy=ny/mag_n*2;
		//	dvx=nx/mag_n*2*10, dvy=ny/mag_n*2*10;
			if(swapped)
				B2->translate_world(Point(travel*dvx, travel*dvy));
			//	B2->p.x+=travel*dvx, B2->p.y+=travel*dvy, B2->update_world_coord();
			else
				A2->translate_world(Point(-travel*dvx, -travel*dvy));
			//	A2->p.x-=travel*dvx, A2->p.y-=travel*dvy, A2->update_world_coord();
		}
	}//*/

	//resolve collision		//http://www.hakenberg.de/diffgeo/collision_resolution.htm
	double rax=cx-A2->p.x, ray=cy-A2->p.y, raxn=rax*ny-ray*nx;
	double rbx=cx-B2->p.x, rby=cy-B2->p.y, rbxn=rbx*ny-rby*nx;
	double lambda=( (A2->v.x-B2->v.x)*nx+(A2->v.y-B2->v.y)*ny + A2->vr*raxn - B2->vr*rbxn )
		/ ( (A2->inv_mass+B2->inv_mass)*(nx*nx+ny*ny) + raxn*raxn*A2->inv_J + rbxn*rbxn*B2->inv_J );
	//double lambda=damping*2*( (A2->v.x-B2->v.x)*nx+(A2->v.y-B2->v.y)*ny + A2->vr*raxn - B2->vr*rbxn )
	//	/ ( (A2->inv_mass+B2->inv_mass)*(nx*nx+ny*ny) + raxn*raxn*A2->inv_J + rbxn*rbxn*B2->inv_J );
	if(info)
	{
		GUIPrint(ghMemDC, cx+5, cy-5, "%lf", lambda);
		draw_rectangle(cx-5, cx+5, cy-5, cy+5);
	}
	if(abs(lambda)<(A.mass<B.mass?A.mass:B.mass)*3&&!(A.mech_unstable=='S'||B.mech_unstable=='S'))
	{
		double lambda_ma=lambda*A2->inv_mass, lambda_mb=lambda*B2->inv_mass;
		A2->v.x-=lambda_ma*nx, A2->v.y-=lambda_ma*ny, A2->vr-=lambda*raxn*A2->inv_J;
		B2->v.x+=lambda_mb*nx, B2->v.y+=lambda_mb*ny, B2->vr+=lambda*rbxn*B2->inv_J;
		return true;
	}
	A.mech_unstable='U', B.mech_unstable='U';
	lambda*=1+damping;
	//lambda*=1;//
	double lambda_ma=lambda*A2->inv_mass, lambda_mb=lambda*B2->inv_mass;
	A2->v.x-=lambda_ma*nx, A2->v.y-=lambda_ma*ny, A2->vr-=lambda*raxn*A2->inv_J;
	B2->v.x+=lambda_mb*nx, B2->v.y+=lambda_mb*ny, B2->vr+=lambda*rbxn*B2->inv_J;
										
	lastlambda=lambda;//
	return false;
}
bool			interpolation_circle_x(Point const &center, double radius2, double y, double &x1, double &x2)
{//x = cx +- sqrt(r2 - (y-cy)2)		y = cy +- sqrt(r2 - (x-cx)2)
	double dy=y-center.y, disc=radius2-dy*dy;
	if(disc>=0)
	{
		disc=sqrt(disc);
		x1=center.x-disc, x2=center.x+disc;
		return true;
	}
	return false;
}
bool			interpolation_circle_y(Point const &center, double radius2, double x, double &y1, double &y2)
{//x = cx +- sqrt(r2 - (y-cy)2)		y = cy +- sqrt(r2 - (x-cx)2)
	double dx=x-center.x, disc=radius2-dx*dx;
	if(disc>=0)
	{
		disc=sqrt(disc);
		y1=center.y-disc, y2=center.y+disc;
		return true;
	}
	return false;
}
bool			intersection_circle_line(Point const &center, double radius2, Point const &p1, Point const &p2, Point &sol1, Point &sol2)
{
	double a=p2.y-p1.y, b=p1.x-p2.x, c=-(a*p1.x+b*p1.y);
	if(!a)//horizontal
	{
		double y=-c/b;
		double x1, x2;
		if(interpolation_circle_x(center, radius2, y, x1, x2))
		{
			sol1.set(x1, y), sol2.set(x2, y);
			return true;
		}
		return false;
	}
	double a2=a*a;
	double A=a2+b*b, B=2*(b*(a*center.x+c)-a2*center.y), C=(center.x*center.x+center.y*center.y-radius2)*a2+2*a*c*center.x+c*c;
	double disc=B*B-4*A*C;
	if(disc>=0)
	{
		disc=sqrt(disc), A*=2;
		sol1.y=(-B-disc)/A, sol2.y=(-B+disc)/A;
		sol1.x=-(b*sol1.y+c)/a, sol2.x=-(b*sol2.y+c)/a;
		return true;
	}
	return false;
}
#ifdef ENGINE_V3
void			resolve_collision_lever(PhysPolygon &A, Point &hinge, CollisionInfo const &c)
{
	//int LOL_1=!inclusion_polygon_point(A, hinge),
	//	LOL_2=hinge.x<xleft,
	//	LOL_3=hinge.x>xright,
	//	LOL_4=hinge.y<yground,
	//	LOL_5=hinge.y>ytop;
	draw_line(hinge.x-5, hinge.y, hinge.x+5, hinge.y), draw_line(hinge.x, hinge.y-5, hinge.x, hinge.y+5);//
//	draw_ellipse(hinge-5, hinge+5);//
	//if(!inclusion_polygon_point(A, hinge)||//X		may miss hinge on face
	//	hinge.x<xleft||hinge.x>xright||hinge.y<yground-0.01||hinge.y>ytop)
	//	return;
	if(A.mech_unstable!='S')
	{
		A.vr+=A.mass*gravity*(hinge.x-A.p.x)*A.inv_J*timescale;
		A.rotate_around(hinge, A.vr*timescale);
	}
	
	//detect & resolve collisions
	//if(scenario_number==2&&frame_number==107&&A.idx==0)
	//	int LOL_1=0;
	for(int k=0, kEnd=polygons.size();k<kEnd;++k)
	{
		if(k!=A.idx)
		{
			auto &B=polygons[k];
			double r=A.r+B.r;
			if(abs(A.p.x-B.p.x)<=r&&abs(A.p.y-B.p.y)<=r)//A & B may collide
			{
				bool As_side, rotational;
				int point, side;
				double travel;
				if(intersection_polygons(A, B, As_side, point, side, travel, rotational))
				{
					if(!(c.idx==B.idx&&c.point==point))
					{
						Point p1, p2;
						if(As_side)
						{
							Point v2=B.w[point]-hinge;
							int side_v2=(side+1)%A.w.size();
							if(intersection_circle_line(hinge, v2.mag_sq(), A.w[side], A.w[side_v2], p1, p2))
							{
								double d_p1=abs(distance_to_line_times_seg_length(A.p, B.p, p1)),
									d_p2=abs(distance_to_line_times_seg_length(A.p, B.p, p2));
								Point p=d_p1<d_p2?p1:p2;
								
								const Point *w1, *w2;
								if(range_y(A.w[side], A.w[side_v2], w1, w2))
								{
									if(p.y<w1->y)
										p=*w1;
									else if(w2->y<p.y)
										p=*w2;
								}
								else
								{
									if(p.x<w1->x)
										p=*w1;
									else if(w2->x<p.x)
										p=*w2;
								}
								Point v3=p-hinge;
								//double d_centroid=distance_to_line_times_seg_length(hinge, B.w[point], B.p),
								//	d_p1=distance_to_line_times_seg_length(hinge, B.w[point], p1);
								//Point v3=((d_centroid>=0)==(d_p1>=0)?p1:p2)-hinge;
								double angle_change=v2.angle2()-v3.angle2();
								A.rotate_around(hinge, angle_change);

								A.vr=-A.vr*damping;
								if(A.vr<0.1)
								{
									A.v.x=0, A.v.y=0;//
									A.vr=0;
									A.mech_unstable='S';
								}
							}
							else
								int LOL_1=0;//
						}
						else//B's side
						{
							Point v2=A.w[point]-hinge;
							int side_v2=(side+1)%B.w.size();
							if(intersection_circle_line(hinge, v2.mag_sq(), B.w[side], B.w[side_v2], p1, p2))
							{
								//if(info)
								//{
								//	double r=v2.magnitude(); draw_ellipse(hinge-r, hinge+r);//
								//	draw_rectangle(p1-10, p1+10), draw_rectangle(p2-10, p2+10);//
								//}
								double d_centroid=distance_to_line_times_seg_length(hinge, A.w[point], A.p),
									d_p1=distance_to_line_times_seg_length(hinge, A.w[point], p1);
								Point p=(d_centroid>=0)==(d_p1>=0)?p1:p2;

								const Point *w1, *w2;
								if(range_y(B.w[side], B.w[side_v2], w1, w2))
								{
									if(p.y<w1->y)
										p=*w1;
									else if(w2->y<p.y)
										p=*w2;
								}
								else
								{
									if(p.x<w1->x)
										p=*w1;
									else if(w2->x<p.x)
										p=*w2;
								}
							/*	if(B.w[side].x==B.w[side_v2].x)
								{
									if(B.w[side].y<B.w[side_v2].y)
										w1=&B.w[side].y, w2=&B.w[side_v2].y;
									else
										w1=&B.w[side_v2].y, w2=&B.w[side].y;
								}
								else
								{
								}//*/
							//	if(inclusion_segment_point(B.w[side], B.w[side_v2], p))
								Point v3=p-hinge;
								double angle_change=v3.angle2()-v2.angle2();
								A.rotate_around(hinge, angle_change);

								A.vr=-A.vr*damping;
								if(A.vr<0.1)
								{
									A.v.x=0, A.v.y=0;//
									A.vr=0;
									A.mech_unstable='S';
								}
							}
							else
								int LOL_1=0;//
						}
					}
				}
			}
		}
	}

	//check & resolve collision with ground
	int point=0;	double t=A.w[0].y;//find lowest vertex
	for(int kv=1, kvEnd=A.w.size();kv<kvEnd;++kv)
		if(t>A.w[kv].y)
			point=kv, t=A.w[kv].y;
	if(A.w[point].y<yground)
	{
		Point v2=A.w[point]-hinge;
		double x1, x2;
		double r2=v2.mag_sq();
		if(r2>1e-12&&interpolation_circle_x(hinge, r2, yground, x1, x2))
		{
			double d_centroid=distance_to_line_times_seg_length(hinge, A.w[point], A.p),
				d_p1=distance_to_line_times_seg_length(hinge, A.w[point], Point(x1, yground));
			//double d_p2=distance_to_line_times_seg_length(hinge, A.w[point], Point(x2, yground));
			double x=(d_centroid>=0)==(d_p1>=0)?x1:x2;
			//if(v2.x>0)
			//{
			//}
			//else
			//{
			//}
			Point v3(x-hinge.x, yground-hinge.y);
			double angle_change=v3.angle2()-v2.angle2();
			A.rotate_around(hinge, angle_change);

			A.vr=-A.vr*damping;
			if(A.vr<0.1)
			{
				A.v.x=0, A.v.y=0;//
				A.vr=0;
				A.mech_unstable='S';
			}
		}
	//	double angle_change=v2.angle();
		//A.rotate_around(hinge, -angle_change);
		//A.vr=-A.vr*damping;
		//if(A.vr<0.1)
		//{
		//	A.v.x=0, A.v.y=0;//
		//	A.vr=0;
		//	A.mech_unstable='S';
		//}
	//	A.translate_world(Point(0, yground-A.w[point].y));
	}
	//check & resolve collision with left wall
	point=0;	t=A.w[0].x;//find leftmost vertex
	for(int kv=1, kvEnd=A.w.size();kv<kvEnd;++kv)
		if(t>A.w[kv].x)
			point=kv, t=A.w[kv].x;
	if(A.w[point].x<xleft)//leftmost vertex inside left wall
	{
		Point v2=A.w[point]-hinge;
		double y1, y2;
		double r2=v2.mag_sq();
		if(r2>1e-12&&interpolation_circle_y(hinge, r2, xleft, y1, y2))
		{
			double d_centroid=distance_to_line_times_seg_length(hinge, A.w[point], A.p),
				d_p1=distance_to_line_times_seg_length(hinge, A.w[point], Point(xleft, y1));
			double y=(d_centroid>=0)==(d_p1>=0)?y1:y2;
			Point v3(xleft-hinge.x, y-hinge.y);
			double angle_change=v3.angle2()-v2.angle2();
			A.rotate_around(hinge, angle_change);

			A.vr=-A.vr*damping;
			if(A.vr<0.1)
			{
				A.v.x=0, A.v.y=0;//
				A.vr=0;
				A.mech_unstable='S';
			}
		}
	}
	//check & resolve collision with right wall
	point=0;	t=A.w[0].x;//find rightmost vertex
	for(int kv=1, kvEnd=A.w.size();kv<kvEnd;++kv)
		if(t<A.w[kv].x)
			point=kv, t=A.w[kv].x;
	if(A.w[point].x>xright)
	{
		Point v2=A.w[point]-hinge;
		double y1, y2;
		double r2=v2.mag_sq();
		if(r2>1e-12&&interpolation_circle_y(hinge, r2, xright, y1, y2))
		{
			double d_centroid=distance_to_line_times_seg_length(hinge, A.w[point], A.p),
				d_p1=distance_to_line_times_seg_length(hinge, A.w[point], Point(xright, y1));
			double y=(d_centroid>=0)==(d_p1>=0)?y1:y2;
			Point v3(xright-hinge.x, y-hinge.y);
			double angle_change=v3.angle2()-v2.angle2();
			A.rotate_around(hinge, angle_change);

			A.vr=-A.vr*damping;
			if(A.vr<0.1)
			{
				A.v.x=0, A.v.y=0;//
				A.vr=0;
				A.mech_unstable='S';
			}
		}
	}
	//check & resolve collision with ceiling
	point=0;	t=A.w[0].y;//find highest vertex
	for(int kv=1, kvEnd=A.w.size();kv<kvEnd;++kv)
		if(t<A.w[kv].y)
			point=kv, t=A.w[kv].y;
	if(A.w[point].y>ytop)
	{
		Point v2=A.w[point]-hinge;
		double x1, x2;
		double r2=v2.mag_sq();
		if(r2>1e-12&&interpolation_circle_x(hinge, r2, ytop, x1, x2))
		{
			double d_centroid=distance_to_line_times_seg_length(hinge, A.w[point], A.p),
				d_p1=distance_to_line_times_seg_length(hinge, A.w[point], Point(x1, ytop));
			double x=(d_centroid>=0)==(d_p1>=0)?x1:x2;
			Point v3(x-hinge.x, ytop-hinge.y);
			double angle_change=v3.angle2()-v2.angle2();
			A.rotate_around(hinge, angle_change);

			A.vr=-A.vr*damping;
			if(A.vr<0.1)
			{
				A.v.x=0, A.v.y=0;//
				A.vr=0;
				A.mech_unstable='S';
			}
		}
	}
}
#endif
inline double	maximum(double a, double b){return 0.5*(a+b+abs(a-b));}
inline double	clamp_positive(double x)//return 0 if x is negative
{
	return 0.5*(x+abs(x));
	//return x<0?0:x;
}
inline double	clamp_negative(double x){return 0.5*(x-abs(x));}//return 0 if x is positive
double			clamp(double x, double min, double max)
{
	double t=0.5*(x+min+abs(x-min));
	return 0.5*(t+max-abs(t-max));
//	return x<min?min:x>max?max:x;
}
int				find_contact(PhysPolygon const &A, PhysPolygon const &B)
{
	for(int kc=0, kcEnd=contacts.size();kc<kcEnd;++kc)//find contact with these objects
	{
		auto &c=contacts[kc];
		if(c.a_idx==A.idx&&c.b_idx==B.idx||c.a_idx==B.idx&&c.b_idx==A.idx)
			return kc;
	}
	return -1;
}
void			insert_contact_point(PhysPolygon const &A, PhysPolygon const &B, Point const &n, Point const &ra, Point const &rb, double depth, int kc, bool &new_contact)
{
	bool persistent=false;
	int kp=0;
	if(warm_starting&&kc!=-1)
	{
		auto &c=contacts[kc];
		for(int kpEnd=c.p.size();kp<kpEnd;++kp)//search for close previous contact
		{
			auto &p=c.p[kp];
			Point dra=ra-p.ra, drb=rb-p.rb;
			if(abs(dra.x)<proximity_limit&&abs(dra.y)<proximity_limit&&abs(drb.x)<proximity_limit&&abs(drb.y)<proximity_limit)
			{
				persistent=true;
				break;
			}
		}
	}
	if(persistent)
	{
		auto &c=contacts[kc];
		c.persistent=true;//
		auto &p=c.p[kp];
		p.travel=depth;
		p.ra=ra, p.rb=rb;
	}
	else if(kc!=-1)
		contacts[kc].add_cp(ra, rb, depth);
	else if(new_contact)
	{
		contacts.push_back(ContactInfo(A.idx, B.idx, depth, ra, rb, n.x, n.y));
		new_contact=false;
	}
	else
		contacts.rbegin()->add_cp(ra, rb, depth);
}
int				find_contact(PhysPolygon const &A, Point const &n)
{
	for(int kc=0, kcEnd=contacts.size();kc<kcEnd;++kc)//find A's contact with boundary
	{
		auto &c=contacts[kc];
		if(c.a_idx==A.idx&&c.b_idx==-2&&c.nx==n.x&&c.ny==n.y)
			return kc;
	}
	return -1;
}
void			insert_contact_point(PhysPolygon const &A, Point const &n, Point const &ra, Point const &rb, double depth, int kc, bool &new_contact)
{
	bool persistent=false;
	int kp=0;
	if(warm_starting&&kc!=-1)//A already has contact with ground
	{
		auto &c=contacts[kc];
		for(int kpEnd=c.p.size();kp<kpEnd;++kp)
		{
			auto &p=c.p[kp];
			Point dra=ra-p.ra,//local
				drb=rb-p.rb;//world, point may have moved on the surface
			if(abs(dra.x)<proximity_limit&&abs(dra.y)<proximity_limit&&abs(drb.dot(n))<proximity_limit)
			{
				persistent=true;
				break;
			}
		}
	}
	if(persistent)
	{
		auto &c=contacts[kc];
		c.persistent=true;//
		auto &p=c.p[kp];
		p.travel=depth, p.ra=ra, p.rb=rb;
	}
	else if(kc!=-1)
		contacts[kc].add_cp(ra, rb, depth);
	else if(new_contact)
	{
		contacts.push_back(ContactInfo(A.idx, -2, depth, ra, rb, n.x, n.y));//normal pointing out of A
		new_contact=false;
	}
	else
		contacts.rbegin()->add_cp(ra, rb, depth);
}
//void			no_collision(PhysPolygon &A, PhysPolygon &B)//remove collision if exists
//{
//	for(int kc=0, kcEnd=A.collisions.size();kc<kcEnd;++kc)
//	{
//		auto &c=A.collisions[kc];
//		if(c.idx==B.idx)
//		{
//			A.collisions.erase(A.collisions.begin()+kc);
//			break;
//		}
//	}
//	for(int kc=0, kcEnd=B.collisions.size();kc<kcEnd;++kc)
//	{
//		auto &c=B.collisions[kc];
//		if(c.idx==A.idx)
//		{
//			B.collisions.erase(B.collisions.begin()+kc);
//			break;
//		}
//	}
//}
//void			no_collision(PhysPolygon &A, double nx, double ny)//remove collision with boundary if exists
//{
//	for(int kc=0, kcEnd=A.collisions.size();kc<kcEnd;++kc)
//	{
//		auto &c=A.collisions[kc];
//		if(c.idx==-1&&c.nx==nx&&c.ny==ny)
//		{
//			A.collisions.erase(A.collisions.begin()+kc);
//			break;
//		}
//	}
//}
//double			friction(double v_rel)
//{
//	return v_rel+((v_rel>0)-(v_rel<0))/(abs(v_rel)+0.5);//does nothing
//}

void			initialize()
{
/*	polygons.resize(1);
	polygons[0].set_position(65*5, 30*5, 0);
	polygons[0].set_properties_triangle_xy(0, -30, 20, 10, -20, 10, 0.1);//0.1/5/5	1
	polygons[0].set_velocity(0, 0, 0);//*/

/*	//int tx=30, ty=10;
	//int tx=20, ty=7;
	int tx=4, ty=4;
	double px=(xright-xleft)/(tx+1), py=(ytop-yground)/(ty+1);
//	polygons.resize(ty*tx);
//	for(int ky=0;ky<ty;++ky)
	polygons.resize(ty*tx);
	for(int ky=0;ky<ty;++ky)
	{
		for(int kx=0;kx<tx;++kx)
		{
			auto &A=polygons[tx*ky+kx];
			A.set_properties_triangle_abc(20, 20, 20, 1);
		//	A.set_position((xright-xleft)/2+kx, (ytop-yground)/2+ky, 0);
			A.set_position(px*(kx+1)+ky, py*(ky+1), 0);
			A.set_velocity(0, 0, 0);
			A.update_world_coord();
		}
	}//*/
/*	polygons.resize(1);
		polygons[0].set_properties_triangle_abc(50, 50, 50, 1./5/5);
		polygons[0].set_position(65*5, 30*5, 0);
		polygons[0].set_velocity(0, 0, 0);//*/
/*	int tx=4, ty=4;
	double px=(xright-xleft)/(tx+1), py=(ytop-yground)/(ty+1);
//	polygons.resize(ty*tx);
//	for(int ky=0;ky<ty;++ky)
	polygons.resize(ty*tx);
	for(int ky=0;ky<ty;++ky)
	{
		for(int kx=0;kx<tx;++kx)
		{
			auto &A=polygons[tx*ky+kx];
			A.set_properties_triangle_abc(20, 20, 20, 1);
			A.set_position(px*(kx+1)+ky, py*(ky+1), 0);
			A.set_velocity(0, 0, 0);
			A.update_world_coord();
		}
	}//*/
/*	polygons.resize(1);
		polygons[0].set_properties_triangle_abc(50, 50, 50, 1./5/5);
		polygons[0].set_position(65*5, 30*5, 0);
		polygons[0].set_velocity(0, 0, 0);//*/

/*	int stack_height=5, pyramid_height=5;//pyramid & stack
	double box_w=50,//150 70 50
		box_h=50,//150 70 50
		separation=5;//10
	polygons0.resize(stack_height+pyramid_height*(pyramid_height+1)/2);
	int k=0;
	for(int kr=pyramid_height;kr>=0;--kr)//row
	{
		for(int kb=0;kb<kr;++kb, ++k)
		{
			auto &A=polygons0[k];
			A.set_properties_rectangle(box_w, box_h, 20);
			A.set_position(100+(box_w+separation)*kb+(box_w+separation)*(pyramid_height-kr)/2, (box_h+separation)*(pyramid_height-kr+0.5), pi);
			A.set_velocity(0, 0, 0);
			A.friction=object_friction;
		}
	}
	for(int k2=0, kEnd=polygons0.size();k<kEnd;++k, ++k2)
	{
		auto &A=polygons0[k];
		A.set_properties_rectangle(box_w, box_h, 0.041);//0.2
		A.set_position(700, (separation+box_h)*(k2+0.5), pi);
		A.set_velocity(0, 0, 0);
		A.friction=object_friction;
	}//*/

/*	int stack_height=5;//stack
	double box_w=50,//150 70 50
		box_h=50,//150 70 50
		separation=5;//10
	polygons0.resize(stack_height);
	for(int k=0;k<stack_height;++k)
	{
		auto &A=polygons0[k];
		A.set_properties_rectangle(box_w, box_h, 0.041);//0.2
		A.set_position(400, (separation+box_h)*(k+0.5), pi);
	//	A.set_position(400, 50+(separation+box_h)*(k+1), pi);
		A.set_velocity(0, 0, 0);
		A.friction=object_friction;
	}//*/

/*	int pyramid_height=5;//pyramid
	double box_w=50,//70
		box_h=50,//50
		separation=2;//10
	polygons0.resize(pyramid_height*(pyramid_height+1)/2);
	for(int k=0, kr=pyramid_height;kr>=0;--kr)//row
	{
		for(int kb=0;kb<kr;++kb, ++k)
		{
			auto &A=polygons0[k];
			A.set_properties_rectangle(box_w, box_h, 20);
			A.set_position(100+(box_w+separation)*kb+(box_w+separation)*(pyramid_height-kr)/2, (box_h+separation)*(pyramid_height-kr+1), pi);
			A.set_velocity(0, 0, 0);
			A.friction=object_friction;
		}
	}//*/

	polygons0.resize(10);//5 10 100
	for(int k=0, kEnd=polygons0.size();k<kEnd;++k)
	{
		auto &A=polygons0[k];
	//	A.mech_unstable='B';
		A.idx=k;
		A.set_properties_triangle_abc(50, 50, 50, 0.1);//0.01
		A.friction=object_friction;
		A.set_position(rand()%(int)(xright-50), rand()%(int)(ytop+1), mod(rand(), pi2));
		A.set_velocity(0, 0, 0);
	}
//	polygons0.resize(3);
	//	//polygons0[0].mech_unstable='B';
	//	polygons0[0].set_properties_triangle_abc(50, 50, 50, 0.01);//0.1/5/5	1	0.1
	//	polygons0[0].set_position(65*5, 30*5, 0);
	//	polygons0[0].set_velocity(0, 0, 0);
	////polygons0[1].mech_unstable='B';
	//polygons0[1].set_properties_triangle_abc(50, 50, 50, 0.01);//0.1/5/5		0.1
	//polygons0[1].set_position(60*5, 10*5, 0);
	//polygons0[1].set_velocity(0, 0, 0);
	//	//polygons0[2].mech_unstable='B';
	//	polygons0[2].set_properties_triangle_abc(70, 70, 70, 0.02);//0.1/5/5	1	10
	//	polygons0[2].set_position(100*5, 50, 0);
	//	polygons0[2].set_velocity(0, 10, 0);
	//polygons0.resize(1);
	polygons0[0].set_properties_rectangle(70, 50, 0.2);//box	0.02	0.1/5/5	1	10
	polygons0[0].set_position(100*5, 50, pi);
	polygons0[0].set_velocity(0, 10, 0);
		polygons0[1].set_properties_ball(70, 7, 0.2);//ball	0.02	0.1/5/5	1	10
		polygons0[1].set_position(65*5, 30*5, 0);
		polygons0[1].set_velocity(0, 10, 0);
	polygons0[2].set_properties_rectangle(70, 7, 0.2);//plank	0.02	0.1/5/5	1	10
	polygons0[2].set_position(60*5, 10*5, 0);
	polygons0[2].set_velocity(0, 0, 0);

/*	polygons.resize(2);
		polygons[0].set_properties_triangle_abc(10, 10, 10, 1);
		polygons[0].set_position(65, 30, 0);
		polygons[0].set_velocity(0, 0, 0);
	polygons[1].set_properties_triangle_abc(10, 10, 10, 1);
	polygons[1].set_position(60, 10, 0);
	polygons[1].set_velocity(0, 0, 0);//*/
/*	int tx=4, ty=4;
	double px=(xright-xleft)/(tx+1), py=(ytop-yground)/(ty+1);
//	polygons.resize(ty*tx);
//	for(int ky=0;ky<ty;++ky)
	polygons.resize(ty*tx);
	for(int ky=0;ky<ty;++ky)
	{
		for(int kx=0;kx<tx;++kx)
		{
			auto &A=polygons[tx*ky+kx];
			A.set_properties_triangle_abc(4, 4, 4, 1);
		//	A.set_properties_triangle_abc(1, 1, 1, rand()%200*0.01);
		//	A.set_properties_triangle_abc(rand()%1000*0.01, rand()%1000*0.01, rand()%1000*0.01, rand()%200*0.01);
			A.set_position(px*(kx+1)+ky, py*(ky+1), 0);
			A.set_velocity(0, 0, 0);
		//	A.set_velocity(rand()%100*0.01, rand()%100*0.01, rand()%100*0.01);
			A.update_world_coord();
		}
	}//*/
	for(int k=0, kEnd=polygons0.size();k<kEnd;++k)//initialize idx & world coordinates
	{
		auto &A=polygons0[k];
		A.idx=k, A.iterate(0);
	}
	polygons=polygons0;
	contacts.resize(0);
}
long __stdcall	WndProc(HWND hWnd, unsigned int message, unsigned int wParam, long lParam)
{
	switch(message)
	{
	case WM_CREATE:
		initialize();
		return 0;
	case WM_PAINT:
		if(!pause&&!timer)
	//	if(!timer)
		{
			timer=true;
			SetTimer(hWnd, 0, 10, 0);
		}
		else//*/
		{
			{
				RECT R;
				GetClientRect(hWnd, &R);
			//	GetWindowRect(hWnd, &R);
				int w2=R.right-R.left, h2=R.bottom-R.top;
				if(w2!=w||h2!=h)
				{
					w=w2, h=h2, Ox=w/2, Oy=h/2;
					DeleteObject(SelectObject(ghMemDC, ghBitmap));
					DeleteDC(ghMemDC), ReleaseDC(hWnd, ghDC);
					ghDC=GetDC(hWnd), ghMemDC=CreateCompatibleDC(ghDC);
					ghBitmap=(HBITMAP)SelectObject(ghMemDC, CreateCompatibleBitmap(ghMemDC, w, h));
				}
			}
		//	xoffset=0, yoffset=0, yground=0, ytop=h, xleft=0, xright=w;
		//	xoffset=10, yoffset=10, yground=0, ytop=h-20, xleft=0, xright=w-20;
			xoffset=10, yoffset=36, yground=0, ytop=h-20-26, xleft=0, xright=w-20;
			Rectangle(ghMemDC, -1, -1, w+1, h+1);//wipe bitmap
			if(tick)//itertion
			{

				//differential engine
			/*	{
					long long t1=__rdtsc();
					double x=0;
					for(int k=0;k<1000;++k)
						x+=cos((double)rand())+sin((double)rand());
					long long t2=__rdtsc();
					GUIPrint(ghMemDC, 0, h-36, "sin cos:  %lf", 1000.*(t2-t1));//faster

					t1=__rdtsc();
					for(int k=0;k<1000;++k)
						x+=atan2((double)rand(), (double)rand());
					t2=__rdtsc();
					GUIPrint(ghMemDC, 0, h-18, "atan2:    %lf", 1000.*(t2-t1));
				}//*/
#if 0
				for(int k=0, kEnd=polygons.size();k<kEnd;++k)
				{
					for(int k2=k+1;k2<kEnd;++k2)
					{
						if(polygons[k].mech_unstable=='B'||polygons[k2].mech_unstable=='B')
						{
							auto &A=polygons[k], &B=polygons[k2];
							{
								RotatedRectangle ra, rb;
								A.get_bounding_block(ra), B.get_bounding_block(rb);
								ra.draw(), rb.draw();//
							}

							//check if AABBs intersect
							Point aa1, aa2, bb1, bb2;
							A.get_AABB(aa1, aa2), B.get_AABB(bb1, bb2);
							Point aBL, aTR, bBL, bTR;
							sort_coordinates(aa1, aa2, aBL, aTR);
							sort_coordinates(bb1, bb2, bBL, bTR);
						//	draw_rectangle(aBL, aTR), draw_rectangle(bBL, bTR);//
							Point temp=aTR+bTR-bBL;
							if(aBL.x<bTR.x&&bTR.x<temp.x&&aBL.y<bTR.y&&bTR.y<temp.y)
							{
							//	GUIPrint(ghMemDC, aBL.x, aTR.y, "i");//

								//get bounding block
								RotatedRectangle ra, rb;
								A.get_bounding_block(ra), B.get_bounding_block(rb);

								//check if bounding blocks intersect
								ra.draw(), rb.draw();//
								if(rotated_rectangle_intersect(ra, rb))
								{
									//GUIPrint(ghMemDC, ra.center.x, ra.center.y, "intersect");//
									//GUIPrint(ghMemDC, rb.center.x, rb.center.y, "intersect");//

									bool As_side, rotational;
									int point, side;
									double travel;

									double d_delta=timescale, delta=d_delta;
									A.iterate(d_delta), B.iterate(d_delta);
									if(intersection_triangles(A, B, As_side, point, side, travel, rotational))
									{
										d_delta/=2, delta-=d_delta;
										A.iterate(-d_delta), B.iterate(-d_delta);
										for(int it=0;it<52;++it)
										{
											d_delta/=2;
											if(intersection_triangles(A, B, As_side, point, side, travel, rotational))
												delta-=d_delta, A.iterate(-d_delta), B.iterate(-d_delta);
											else
												delta+=d_delta, A.iterate(d_delta), B.iterate(d_delta);
										}
										GUIPrint(ghMemDC, ra.center.x, ra.center.y, "delta=%lf", delta);//
										GUIPrint(ghMemDC, rb.center.x, rb.center.y, "delta=%lf", delta);//
									}
									A.iterate(-delta), B.iterate(-delta);
								}
							}
						}
					}
				}
#endif

				//granular engine v4 v5
#if 1
				for(int k=0, kEnd=polygons.size();k<kEnd;++k)//simulate
				{
					auto &A=polygons[k];
				//	double mag_momentum=A.mass*A.v.mag_sq()+abs(A.J*A.vr);
					if(A.mech_unstable=='U')
					{
						A.v*=air_viscosity, A.vr*=air_viscosity;
						A.iterate(timescale);
						//if(info)
						//{
						//	HPEN hPen=CreatePen(PS_DOT, 1, 0xFF0000);
						//	hPen=(HPEN)SelectObject(ghMemDC, hPen);
						//	draw_line(A.p.x, A.p.y, A.w[0].x, A.w[0].y);
						//	A.draw();
						//	hPen=(HPEN)SelectObject(ghMemDC, hPen);
						//	DeleteObject(hPen);
						//}
					}
					//if(A.mech_unstable=='S'&&(A.v.x!=0||A.v.y!=0||A.vr!=0))
					//	int LOL_1=0;
				}
#if defined ENGINE_V4||defined ENGINE_V5
				//if(frame_number==5)
				//if(frame_number==2)
				//if(frame_number==12)
				//	int LOL_1=0;
				if(warm_starting)
				{
					for(unsigned kc=0;kc<contacts.size();)//remove invalid contacts
					{
						auto &c=contacts[kc];
						switch(c.b_idx)
						{
						case -2://boundary
							{
								auto &A=polygons[c.a_idx];
							//	bool no_collision=false;
								for(unsigned kp=0;kp<c.p.size();)
								{
									auto &p=c.p[kp];
#ifdef LOCAL_COORDINATES
									double depth=(A.local_to_world(p.ra)-p.rb).dot(Point(c.nx, c.ny));
#else
									double depth=(A.p+p.ra-p.rb).dot(Point(c.nx, c.ny));
#endif
									if(depth<=0)//negative depth: no penetration
										c.p.erase(c.p.begin()+kp);
									else
										++kp;
								}
								if(c.p.size())
								{
									c.persistent=true;//all remaining old contacts are persistent
									++kc;
								}
								else
									contacts.erase(contacts.begin()+kc);
							}
							break;
						case -1://nothing
							contacts.erase(contacts.begin()+kc);
							break;
						default://object
							{
								auto &A=polygons[c.a_idx], &B=polygons[c.b_idx];
								for(unsigned kp=0;kp<c.p.size();)
								{
									auto &p=c.p[kp];
#ifdef LOCAL_COORDINATES
									Point pa=A.local_to_world(p.ra), pb=B.local_to_world(p.rb);
									double depth=(pa-pb).dot(Point(c.nx, c.ny));
#else
									double depth=(A.p+p.ra-(B.p+p.rb)).dot(Point(c.nx, c.ny));
#endif
									if(depth<=0||abs(pa.x-B.p.x)>B.r||abs(pa.y-B.p.y)>B.r||abs(pb.x-A.p.x)>A.r||abs(pb.y-A.p.y)>A.r)//negative depth: no penetration
								//	if(depth<=0)
										c.p.erase(c.p.begin()+kp);
									else
									{
										c.persistent=true;
										++kp;
									}
								}
								double r=A.r+B.r;
								if(c.p.size()&&abs(A.p.x-B.p.x)<=r&&abs(A.p.y-B.p.y)<=r)
									++kc;
								else
									contacts.erase(contacts.begin()+kc);
							}
							break;
						}
					}
				}
				for(int ka=0, kaEnd=polygons.size()-1;ka<kaEnd;++ka)//BROADPHASE & NARROWPHASE: detect new contacts
				{
					auto &A=polygons[ka];
				//	Point aBL, aTR;
				//	A.get_AABB_sorted(aBL, aTR);
				//	draw_rectangle(aBL, aTR);//
					for(int kb=ka+1, kbEnd=polygons.size();kb<kbEnd;++kb)
					{
						auto &B=polygons[kb];
						if(A.mech_unstable=='U'||B.mech_unstable=='U')
						{
							A.mech_unstable='U', B.mech_unstable='U';
						//	Point bBL, bTR;
						//	B.get_AABB_sorted(bBL, bTR);
						////	draw_rectangle(bBL, bTR);//
						//	Point temp=aTR+bTR-bBL;
						//	if(aBL.x<bTR.x&&bTR.x<temp.x&&aBL.y<bTR.y&&bTR.y<temp.y)//A & B may have intersected
							double r=A.r+B.r;
							if(abs(A.p.x-B.p.x)<=r&&abs(A.p.y-B.p.y)<=r)//A & B may collide
							{
							//	Simplex s;

							//	long long t1=__rdtsc();
							//	//bool condition=intersection_polygons_GJK_website(A, B, A.p-B.p, s);	//A-B: 1.35 1.425 1.27	B-A: 2.30
							//	bool condition=intersection_polygons_GJK(A, B, B.p-A.p, s);			//A-B: 1.63				B-A: 1.43
							//	long long t2=__rdtsc();
							//	static double t_total=0;
							//	static int count=0;
							//	t_total+=(t2-t1)/1000., ++count;
							//	GUIPrint(ghMemDC, w/2, h/2-18, "%lf", t_total/count);

								if(collision_detection_SAT)
								{
									double depth;
									Point normal, cp[2];
									int count;
									if(intersection_SAT(A, B, depth, normal, cp, count))
									{
#ifdef LOCAL_COORDINATES
										Point ra0=A.world_to_local(cp[0]), rb0=B.world_to_local(cp[0]),
											ra1=A.world_to_local(cp[1]), rb1=B.world_to_local(cp[1]);
#else
										Point ra0=cp[0]-A.p, rb0=cp[0]-B.p, ra1=cp[1]-A.p, rb1=cp[1]-B.p;
#endif
										bool new_contact=true;
										int kc=find_contact(A, B);
										insert_contact_point(A, B, normal, ra0, rb0, depth*0.1, kc, new_contact);
										insert_contact_point(A, B, normal, ra1, rb1, depth*0.1, kc, new_contact);
									}
								}
								else
								{
									Point normal,//pointing out of A
										ca, cb;
									double depth;
									bool not_sure;
									//if(frame_number==32&&A.idx==3&&B.idx==9)
									//	int LOL_1=0;
								//	long long t1=__rdtsc();
									if(intersection_EPA(A, B, normal, depth, not_sure, ca, cb))
								//	if(condition)
								//	if(intersection_polygons_GJK_website(A, B, A.p-B.p, s))
								//	if(intersection_polygons_GJK(A, B, A.p-B.p, s))
									{
										//long long t2=__rdtsc();
										//GUIPrint(ghMemDC, w/2, h/2, "EPA: %lf", (t2-t1)/1000.);
										//{
										//	double depth;
										//	Point normal, cp[2];
										//	int count;
										//	t1=__rdtsc();
										//	if(intersection_SAT(A, B, depth, normal, cp, count))
										//	{
										//	}
										//	t2=__rdtsc();
										//	GUIPrint(ghMemDC, w/2, h/2+18, "SAT: %lf", (t2-t1)/1000.);
										//}

										//{
										//	double depth;
										//	Point normal;
										//	intersection_BF(A, B, depth, normal, cp, count);
										//}
										//if(info)
										//{
										//	GUIPrint(ghMemDC, A.p.x, A.p.y, not_sure?"not_sure":"intersect");//
										//	//GUIPrint(ghMemDC, B.p.x, B.p.y, not_sure?"not_sure":"intersect");//
										//	//Point ca=support(A, normal), cb=support(B, -normal);//X
										//	draw_line(ca, ca+100*normal);
										//	//draw_line(cb, cb-100*normal);//
										//	//draw_line(ca.x-5, ca.y, ca.x+5, ca.y), draw_line(ca.x, ca.y-5, ca.x, ca.y+5);//
										//	//draw_line(cb.x-5, cb.y, cb.x+5, cb.y), draw_line(cb.x, cb.y-5, cb.x, cb.y+5);//
										//}
#ifdef LOCAL_COORDINATES
										Point ra=A.world_to_local(ca), rb=B.world_to_local(cb);
#else
										Point ra=ca-A.p, rb=cb-B.p;
#endif
										bool new_contact=true;
										int kc=find_contact(A, B);
										insert_contact_point(A, B, normal, ra, rb, depth, kc, new_contact);
									}
								}
							}
						}
					}
				}
				//if(frame_number==2)
				//	int LOL_1=0;
				for(int k=0, kEnd=polygons.size();k<kEnd;++k)//detect contacts with boundaries
				{
					auto &A=polygons[k];
				//	if(A.mech_unstable!='S')
				//	{
						//double max_travel=0;
						if(A.p.y-A.r<yground)//ground
						{
							Point n_ground(0, -1);//pointing out of A
							int kc=find_contact(A, n_ground);
							bool new_contact=kc==-1;
							for(int kv=0, kvEnd=A.w.size();kv<kvEnd;++kv)
							{
								if(A.w[kv].y<yground)
								{
									double travel=yground-A.w[kv].y;
#ifdef LOCAL_COORDINATES
									Point ra=A.world_to_local(A.w[kv]), rb(A.w[kv].x, yground);
#else
									Point ra=A.w[kv]-A.p, rb(A.w[kv].x, yground);
#endif
									insert_contact_point(A, n_ground, ra, rb, travel, kc, new_contact);
								}
							}
						}
				//	}
					if(A.p.x-A.r<xleft)//left wall
					{
						Point n_left(-1, 0);//pointing out of A
						int kc=find_contact(A, n_left);
						bool new_contact=kc==-1;
						for(int kv=0, kvEnd=A.w.size();kv<kvEnd;++kv)
							if(A.w[kv].x<xleft)
#ifdef LOCAL_COORDINATES
								insert_contact_point(A, n_left, A.world_to_local(A.w[kv]), Point(xleft, A.w[kv].y), xleft-A.w[kv].x, kc, new_contact);
#else
								insert_contact_point(A, n_left, A.w[kv]-A.p, Point(xleft, A.w[kv].y), xleft-A.w[kv].x, kc, new_contact);
#endif
					}
					if(A.p.x+A.r>xright)//right wall
					{
						Point n_right(1, 0);//pointing out of A
						int kc=find_contact(A, n_right);
						bool new_contact=kc==-1;
						for(int kv=0, kvEnd=A.w.size();kv<kvEnd;++kv)
							if(A.w[kv].x>xright)
#ifdef LOCAL_COORDINATES
								insert_contact_point(A, n_right, A.world_to_local(A.w[kv]), Point(xright, A.w[kv].y), A.w[kv].x-xright, kc, new_contact);
#else
								insert_contact_point(A, n_right, A.w[kv]-A.p, Point(xright, A.w[kv].y), A.w[kv].x-xright, kc, new_contact);
#endif
					}
					if(A.p.y+A.r>ytop)//ceiling
					{
						Point n_top(0, 1);//pointing out of A
						int kc=find_contact(A, n_top);
						bool new_contact=kc==-1;
						for(int kv=0, kvEnd=A.w.size();kv<kvEnd;++kv)
							if(A.w[kv].y>ytop)
#ifdef LOCAL_COORDINATES
								insert_contact_point(A, n_top, A.world_to_local(A.w[kv]), Point(A.w[kv].x, ytop), A.w[kv].y-ytop, kc, new_contact);
#else
								insert_contact_point(A, n_top, A.w[kv]-A.p, Point(A.w[kv].x, ytop), A.w[kv].y-ytop, kc, new_contact);
#endif
					}
					//if(A.mech_unstable!='S'&&!A.resting_on.size())
					//	A.mech_unstable='U';
				}
				for(int k=0, kEnd=polygons.size();k<kEnd;++k)//apply external forces
				{
					auto &A=polygons[k];
					A.v.y-=gravity*timescale;
				}
				//if(frame_number==2)
				//	int LOL_1=0;
				int point=0;
				if(info)
					GUIPrint(ghMemDC, 0, h/2-18, "mass_n, mass_t, bias");
				for(int kc=0, kcEnd=contacts.size();kc<kcEnd;++kc)//resolve contacts: pre step
				{
					auto &c=contacts[kc];
					for(int kp=0, kpEnd=c.p.size();kp<kpEnd;++kp, ++point)
					{
						auto &p=c.p[kp];
						if(c.b_idx==-2)
						{
							auto &A=polygons[c.a_idx];
							Point n(c.nx, c.ny), t(c.ny, -c.nx);
#ifdef LOCAL_COORDINATES
							Point ra=A.local_to_world_rotate_only(p.ra);
#else
							Point &ra=p.ra;
#endif
							double ra_n=ra.dot(n), ra_t=ra.dot(t), ra2=ra.dot(ra);
							p.mass_n=1/(A.inv_mass+(ra2-ra_n*ra_n)*A.inv_J);
							p.mass_t=1/(A.inv_mass+(ra2-ra_t*ra_t)*A.inv_J);
							p.bias=-beta/timescale*clamp_negative(-p.travel+penetration_slop);
							if(info)
								GUIPrint(ghMemDC, 0, h/2+18*point, "%.2lf, %.2lf, %.2lf", p.mass_n, p.mass_t, p.bias);//
							if(accumulated_impulses)
							{
								Point P=p.Pn*n+p.Pt*t;
								A.v-=A.inv_mass*P,		A.vr-=A.inv_J*ra.cross(P);
							}
						}
						else if(c.b_idx>=0)
						{
							auto &A=polygons[c.a_idx], &B=polygons[c.b_idx];
							Point n(c.nx, c.ny), t(c.ny, -c.nx);
#ifdef LOCAL_COORDINATES
							Point ra=A.local_to_world_rotate_only(p.ra), rb=B.local_to_world_rotate_only(p.rb);
#else
							Point &ra=p.ra, &rb=p.rb;
#endif
							double ra_n=ra.dot(n), rb_n=rb.dot(n), ra_t=ra.dot(t), rb_t=rb.dot(t);
							double ma_mb=A.inv_mass+B.inv_mass,
								ra2=ra.dot(ra), rb2=rb.dot(rb);
							p.mass_n=1/(ma_mb+(ra2-ra_n*ra_n)*A.inv_J+(rb2-rb_n*rb_n)*B.inv_J);
							p.mass_t=1/(ma_mb+(ra2-ra_t*ra_t)*A.inv_J+(rb2-rb_t*rb_t)*B.inv_J);
							p.bias=-beta/timescale*clamp_negative(-p.travel+penetration_slop);
							if(info)
								GUIPrint(ghMemDC, 0, h/2+18*point, "%.2lf, %.2lf, %.2lf", p.mass_n, p.mass_t, p.bias);//
							if(accumulated_impulses)
							{
								Point P=p.Pn*n+p.Pt*t;
								A.v-=A.inv_mass*P,		A.vr-=A.inv_J*ra.cross(P);
								A.v+=B.inv_mass*P,		B.vr+=B.inv_J*rb.cross(P);
							}
						}
					}
				}
				for(int i=0;i<n_iterations;++i)//apply impulses
				{
					for(int kc=0, kcEnd=contacts.size();kc<kcEnd;++kc)
					{
						auto &c=contacts[kc];
						for(int kp=0, kpEnd=c.p.size();kp<kpEnd;++kp)
						{
							auto &p=c.p[kp];
							if(c.b_idx==-2)
							{
								auto &A=polygons[c.a_idx];
								Point n(c.nx, c.ny), t(c.ny, -c.nx);
#ifdef LOCAL_COORDINATES
								Point ra=A.local_to_world_rotate_only(p.ra);
#else
								Point &ra=p.ra;
#endif
								Point dv=-A.v-cross(A.vr, ra);
								double vn=dv.dot(n);
								double dPn=p.mass_n*(-vn+p.bias);
								if(accumulated_impulses)
								{
									double Pn0=p.Pn;
									p.Pn=clamp_positive(Pn0+dPn);
									dPn=p.Pn-Pn0;
								}
								else
									dPn=clamp_positive(dPn);
								Point Pn=dPn*n;
								A.v-=Pn*A.inv_mass,		A.vr-=ra.cross(Pn)*A.inv_J;

								dv=-A.v-cross(A.vr, ra);
								double vt=dv.dot(t);
								double f_limit=sqrt(boundary_friction*A.friction)*dPn;
								double dPt=p.mass_t*-vt;
								if(accumulated_impulses)
								{
									double oldTangentImpulse=p.Pt;
									p.Pt=clamp(oldTangentImpulse+dPt, -f_limit, f_limit);
									dPt=p.Pt-oldTangentImpulse;
								}
								else
									dPt=clamp(dPt, -f_limit, f_limit);
								Point Pt=dPt*t;
								A.v-=Pt*A.inv_mass,		A.vr-=ra.cross(Pt)*A.inv_J;

								if(info)
								{
									Point cp=A.local_to_world(p.ra);
									draw_line(cp, cp-(Pn+Pt)*0.1);//
								}
							}
							else if(c.b_idx>=0)
							{
								auto &A=polygons[c.a_idx], &B=polygons[c.b_idx];
								Point n(c.nx, c.ny), t(c.ny, -c.nx);
#ifdef LOCAL_COORDINATES
								Point ra=A.local_to_world_rotate_only(p.ra), rb=B.local_to_world_rotate_only(p.rb);
#else
								Point &ra=p.ra, &rb=p.rb;
#endif
								Point dv=B.v+cross(B.vr, rb)-A.v-cross(A.vr, ra);
								double vn=dv.dot(n);
								double dPn=p.mass_n*(-vn+p.bias);
								if(accumulated_impulses)
								{
									double Pn0=p.Pn;
									p.Pn=clamp_positive(Pn0+dPn);
									dPn=p.Pn-Pn0;
								}
								else
									dPn=clamp_positive(dPn);
								Point Pn=dPn*n;
								A.v-=Pn*A.inv_mass,		A.vr-=ra.cross(Pn)*A.inv_J;
								B.v+=Pn*B.inv_mass,		B.vr+=rb.cross(Pn)*B.inv_J;

								dv=B.v+cross(B.vr, rb)-A.v-cross(A.vr, ra);
								double vt=dv.dot(t);
								double f_limit=sqrt(A.friction*B.friction)*dPn;
								double dPt=-vt*p.mass_t;
								if(accumulated_impulses)
								{
									double oldTangentImpulse=p.Pt;
									p.Pt=clamp(oldTangentImpulse+dPt, -f_limit, f_limit);
									dPt=p.Pt-oldTangentImpulse;
								}
								else
									dPt=clamp(dPt, -f_limit, f_limit);
								Point Pt=dPt*t;
								A.v-=Pt*A.inv_mass,		A.vr-=ra.cross(Pt)*A.inv_J;
								B.v+=Pt*B.inv_mass,		B.vr+=rb.cross(Pt)*B.inv_J;
								if(info)
								{
									Point cpa=A.local_to_world(p.ra), cpb=B.local_to_world(p.rb);//
									//if(!inclusion_polygon_point(A, cpa)||!inclusion_polygon_point(B, cpb))
									//	int LOL_1=0;
									draw_line(cpa, cpa-(Pn+Pt), 0x00FF00), draw_line(cpb, cpb+(Pn+Pt), 0x0000FF);//
								}
							}
						}
					}
				}
#if 0
				for(int kc=0, kcEnd=contacts.size();kc<kcEnd;++kc)//resolve contacts
				{
					auto &c=contacts[kc];
					//int p_idx[2]={0, 1}, p_count=0;//indeces of deepest & farthest points
					//if(c.p.size()<=2)
					//	p_count=c.p.size();
					//else//ignore all but the deepest and farthest points for A
					//{
					//	p_count=2, p_idx[0]=0;
					//	for(unsigned kp=1;kp<c.p.size();++kp)//find deepest point
					//		if(c.p[p_idx[0]].travel<c.p[kp].travel)
					//			p_idx[0]=kp;
					//	double max_d;
					//	bool uninitialized=true;
					//	for(unsigned kp=0;kp<c.p.size();++kp)//find the farthest point
					//	{
					//		if(kp!=p_idx[0])
					//		{
					//			if(uninitialized)
					//				max_d=(c.p[kp].ra-c.p[p_idx[0]].ra).mag_sq(), p_idx[1]=kp, uninitialized=false;
					//			else
					//			{
					//				double d=(c.p[kp].ra-c.p[p_idx[0]].ra).mag_sq();
					//				if(max_d<d)
					//					max_d=d, p_idx[1]=kp;
					//			}
					//		}
					//	}
					//}
					switch(c.b_idx)
					{
					case -2://boundary
						{
							auto &A=polygons[c.a_idx];
							//for(int kp=0;kp<p_count;++kp)
							//{
							//	auto &p=c.p[p_idx[kp]];
							for(int kp=0, kpEnd=c.p.size();kp<kpEnd;++kp)
							{
								auto &p=c.p[kp];
								double &nx=c.nx, &ny=c.ny, tx=ny, ty=-nx;
#ifdef LOCAL_COORDINATES
								Point ra=A.local_to_world_rotate_only(p.ra);
#else
								Point &ra=p.ra;
#endif
							//	draw_line(A.p, ra);//

								double ra_x_n=ra.x*ny-ra.y*nx,
									ra_x_t=ra.x*ty-ra.y*tx;
								double invM_JnT[3]={-  nx*A.inv_mass, -  ny*A.inv_mass, -ra_x_n*A.inv_J};//row vector
								double invM_JtT[3]={-  tx*A.inv_mass, -  ty*A.inv_mass, -ra_x_t*A.inv_J};
								double eff_mass_n=invM_JnT[0]*-nx + invM_JnT[1]*-ny + invM_JnT[2]*-ra_x_n;
								double eff_mass_t=invM_JtT[0]*-tx + invM_JtT[1]*-ty + invM_JtT[2]*-ra_x_t;
								if(p.old)
								{
									A.v.x+=(invM_JnT[0]*p.lambda_n+invM_JtT[0]*p.lambda_t)*old_lambda_factor;
									A.v.y+=(invM_JnT[1]*p.lambda_n+invM_JtT[1]*p.lambda_t)*old_lambda_factor;
									A.vr +=(invM_JnT[2]*p.lambda_n+invM_JtT[2]*p.lambda_t)*old_lambda_factor;
								}
								{
									p.old=true;
									double normalImpulseSum=0, tangentImpulseSum=0;
#ifdef ITERATIONS
									for(int it=0;it<n_iterations;++it)
									{
										double oldvx=A.v.x, oldvy=A.v.y, oldvr=A.vr;
#endif
										double Jn_V1=-nx*A.v.x - ny*A.v.y - ra_x_n*A.vr;
										double Jt_V1=-tx*A.v.x - ty*A.v.y - ra_x_t*A.vr;
										double bn=-beta/timescale*clamp_positive(p.travel-penetration_slop)+C_R*clamp_positive((nx*(-A.v.x + A.vr*ra.y) + ny*(-A.v.y - A.vr*ra.x) )-restitution_slop);
										double bt=-beta/timescale*clamp_positive(p.travel-penetration_slop);//0
										p.lambda_n=-(Jn_V1+bn)/eff_mass_n;
									//	p.lambda_t=-friction(Jt_V1+bt)/eff_mass_t;
										p.lambda_t=-(Jt_V1+bt)/eff_mass_t;

										double Pn=normalImpulseSum;//clamp normal impulse
										normalImpulseSum=clamp_positive(normalImpulseSum+p.lambda_n);
										p.lambda_n=normalImpulseSum-Pn;

										double C_F=sqrt(A.friction*boundary_friction);
										double Pt=tangentImpulseSum;//clamp tangential impulse
										tangentImpulseSum=clamp(tangentImpulseSum+p.lambda_t, -C_F*p.lambda_n, C_F*p.lambda_n);
										p.lambda_t=tangentImpulseSum-Pt;

										A.v.x+=invM_JnT[0]*p.lambda_n+invM_JtT[0]*p.lambda_t;
										A.v.y+=invM_JnT[1]*p.lambda_n+invM_JtT[1]*p.lambda_t;
										A.vr +=invM_JnT[2]*p.lambda_n+invM_JtT[2]*p.lambda_t;
#ifdef ITERATIONS
										if(abs(oldvx-A.v.x)<v_convergence&&abs(oldvy-A.v.y)<v_convergence&&abs(oldvr-A.vr)<v_convergence)
											break;
									}
#endif
									p.lambda_n=normalImpulseSum, p.lambda_t=tangentImpulseSum;
								}
							}
						}
						break;
					case -1:
						break;
					default:
						{
							auto &A=polygons[c.a_idx], &B=polygons[c.b_idx];
							//for(int kp=0;kp<p_count;++kp)
							//{
							//	auto &p=c.p[p_idx[kp]];
							for(int kp=0, kpEnd=c.p.size();kp<kpEnd;++kp)
							{
								auto &p=c.p[kp];
								double tx=c.ny, ty=-c.nx;
#ifdef LOCAL_COORDINATES
								Point ra=A.local_to_world_rotate_only(p.ra), rb=B.local_to_world_rotate_only(p.rb);
#else
								Point &ra=p.ra, &rb=p.rb;
#endif
								double
									ra_x_n=ra.x*c.ny-ra.y*c.nx,
									ra_x_t=ra.x*  ty-ra.y*  tx,
									rb_x_n=rb.x*c.ny-rb.y*c.nx,
									rb_x_t=rb.x*  ty-rb.y*  tx;
								double invM_JnT[6]={-c.nx*A.inv_mass, -c.ny*A.inv_mass, -ra_x_n*A.inv_J, c.nx*B.inv_mass, c.ny*B.inv_mass, rb_x_n*B.inv_J};
								double invM_JtT[6]={-  tx*A.inv_mass, -  ty*A.inv_mass, -ra_x_t*A.inv_J,   tx*B.inv_mass,   ty*B.inv_mass, rb_x_t*B.inv_J};
								if(p.old)
								{
									A.v.x+=(invM_JnT[0]*p.lambda_n+invM_JtT[0]*p.lambda_t)*old_lambda_factor;
									A.v.y+=(invM_JnT[1]*p.lambda_n+invM_JtT[1]*p.lambda_t)*old_lambda_factor;
									A.vr +=(invM_JnT[2]*p.lambda_n+invM_JtT[2]*p.lambda_t)*old_lambda_factor;
									B.v.x+=(invM_JnT[3]*p.lambda_n+invM_JtT[3]*p.lambda_t)*old_lambda_factor;
									B.v.y+=(invM_JnT[4]*p.lambda_n+invM_JtT[4]*p.lambda_t)*old_lambda_factor;
									B.vr +=(invM_JnT[5]*p.lambda_n+invM_JtT[5]*p.lambda_t)*old_lambda_factor;
								}
								{
									p.old=true;
									double eff_mass_n=invM_JnT[0]*-c.nx + invM_JnT[1]*-c.ny + invM_JnT[2]*-ra_x_n + invM_JnT[3]*c.nx + invM_JnT[4]*c.ny + invM_JnT[5]*rb_x_n;
									double eff_mass_t=invM_JtT[0]*-  tx + invM_JtT[1]*-  ty + invM_JtT[2]*-ra_x_t + invM_JtT[3]*  tx + invM_JtT[4]*  ty + invM_JtT[5]*rb_x_t;
									double normalImpulseSum=0, tangentImpulseSum=0;
#ifdef ITERATIONS
									for(int it=0;it<n_iterations;++it)
									{
										double Aoldvx=A.v.x, Aoldvy=A.v.y, Aoldvr=A.vr, Boldvx=B.v.x, Boldvy=B.v.y, Boldvr=B.vr;
#endif
									//	draw_line(A.p, ra), draw_line(B.p, rb);//

										double Jn_V1=-c.nx*A.v.x - c.ny*A.v.y - ra_x_n*A.vr + c.nx*B.v.x + c.ny*B.v.y + rb_x_n*B.vr;
										double Jt_V1=-  tx*A.v.x -   ty*A.v.y - ra_x_t*A.vr +   tx*B.v.x +   ty*B.v.y + rb_x_t*B.vr;
										double bn=-beta/timescale*clamp_positive(p.travel-penetration_slop)+C_R*clamp_positive(( c.nx*(-A.v.x + A.vr*ra.y + B.v.x - B.vr*rb.y) + c.ny*(-A.v.y - A.vr*ra.x + B.v.y + B.vr*rb.x) )-restitution_slop);
										double bt=-beta/timescale*clamp_positive(p.travel-penetration_slop);//0
										p.lambda_n=-(Jn_V1+bn)/eff_mass_n;
									//	p.lambda_t=-friction(Jt_V1+bt)/eff_mass_t;
										p.lambda_t=-(Jt_V1+bt)/eff_mass_t;

										double Pn=normalImpulseSum;//clamp normal impulse
										normalImpulseSum=clamp_positive(normalImpulseSum+p.lambda_n);
										p.lambda_n=normalImpulseSum-Pn;
										//double Pn=clamp_positive(lambda_n);

										double C_F=sqrt(A.friction*B.friction);
										double Pt=tangentImpulseSum;//clamp tangential impulse
										tangentImpulseSum=clamp(tangentImpulseSum+p.lambda_t, -C_F*p.lambda_n, C_F*p.lambda_n);
										p.lambda_t=tangentImpulseSum-Pt;
										//double Pt=clamp(lambda_t, -C_F*Pn, C_F*Pn);

										A.v.x+=invM_JnT[0]*p.lambda_n+invM_JtT[0]*p.lambda_t;
										A.v.y+=invM_JnT[1]*p.lambda_n+invM_JtT[1]*p.lambda_t;
										A.vr +=invM_JnT[2]*p.lambda_n+invM_JtT[2]*p.lambda_t;
										B.v.x+=invM_JnT[3]*p.lambda_n+invM_JtT[3]*p.lambda_t;
										B.v.y+=invM_JnT[4]*p.lambda_n+invM_JtT[4]*p.lambda_t;
										B.vr +=invM_JnT[5]*p.lambda_n+invM_JtT[5]*p.lambda_t;
#ifdef ITERATIONS
										if(abs(Aoldvx-A.v.x)<v_convergence&&abs(Aoldvy-A.v.y)<v_convergence&&abs(Aoldvr-A.vr)<v_convergence
											&&abs(Boldvx-B.v.x)<v_convergence&&abs(Boldvy-B.v.y)<v_convergence&&abs(Boldvr-B.vr)<v_convergence)
											break;
									}
#endif
									p.lambda_n=normalImpulseSum, p.lambda_t=tangentImpulseSum;
								}
							}
						}
						break;
					}
				}
#endif
				
			//	if(info)
			//	{
					//HPEN hPen=CreatePen(PS_DOT, 1, 0xFF0000);
					//hPen=(HPEN)SelectObject(ghMemDC, hPen);
					//draw_line(A.p, ca), draw_line(B.p, cb);
					for(unsigned kc=0;kc<contacts.size();++kc)//draw contacts
					{
						auto &c=contacts[kc];
						if(c.b_idx==-2)//with boundary
						{
							auto &A=polygons[c.a_idx];
							for(unsigned kp=0;kp<c.p.size();++kp)
							{
								Point pa=A.local_to_world(c.p[kp].ra);
								draw_rectangle(pa-2, pa+2);
								if(info)
									GUIPrint(ghMemDC, pa.x, pa.y, kp);
							}
							//	draw_line(A.p, A.local_to_world(c.p[kp].ra));
						}
						else if(c.b_idx>=0)//with object
						{
							auto &A=polygons[c.a_idx], &B=polygons[c.b_idx];
							for(unsigned kp=0;kp<c.p.size();++kp)
							{
								auto &p=c.p[kp];
								Point pa=A.local_to_world(p.ra), pb=B.local_to_world(p.rb);
								draw_rectangle(pa-2, pa+2), draw_rectangle(pb-2, pb+2);
								if(info)
								{
									GUIPrint(ghMemDC, pa.x, pa.y, kp);
									GUIPrint(ghMemDC, pb.x, pb.y, kp);
								}
							//	draw_line(A.p, A.local_to_world(p.ra));
							//	draw_line(B.p, B.local_to_world(p.rb));
							}
						}
					}
					//hPen=(HPEN)SelectObject(ghMemDC, hPen);
					//DeleteObject(hPen);
			//	}
				if(sleeping_enabled)
				{
					for(int k=0, kEnd=polygons.size();k<kEnd;++k)
					{
						auto &A=polygons[k];
						double dp=A.v.mag_sq()*timescale, da=abs(A.vr)*timescale;
						if(dp<min_dp&&da<min_da)
							A.sleep();
					}
				}
				int npoints=0;
				for(unsigned kc=0;kc<contacts.size();++kc)
					npoints+=contacts[kc].p.size();
#endif

				//v3
#ifdef ENGINE_V3
			//	if(frame_number==49)
			//	if(frame_number==362)
				//if(scenario_number==8&&frame_number==265)
			//	if(frame_number==69)
				//if(frame_number==346)
				//if(frame_number==55)
				//if(frame_number==85)
				//if(frame_number==306)
				//if(scenario_number==1&&frame_number==63)
				//if(frame_number==95)
				//if(frame_number==17)
				//	int LOL_1=0;
				for(int ka=0, kaEnd=polygons.size()-1;ka<kaEnd;++ka)//detect collisions
				{
					auto &A=polygons[ka];
					for(int kb=ka+1, kbEnd=polygons.size();kb<kbEnd;++kb)
					{
						auto &B=polygons[kb];
						double r=A.r+B.r;
						if(abs(A.p.x-B.p.x)<=r&&abs(A.p.y-B.p.y)<=r//A & B may collide
						//	&&!(A.mech_unstable=='S'&&B.mech_unstable=='S')//A & B are not static
							)
						{
							bool As_side, rotational;
							int point, side;
							double travel;
							if(intersection_polygons(A, B, As_side, point, side, travel, rotational))
						//	if(intersection_triangles(A, B, As_side, point, side, travel, rotational))
							{
								//if(frame_number==78)
								//if((A.idx==5||A.idx==8)&&(B.idx==5||B.idx==8))
								//if(A.idx==0||A.idx==1||B.idx==0||B.idx==1)
								//	int LOL_1=0;
								//if(side<0||side>2)//
								if(side<0||side>=(int)(As_side?A:B).w.size())//
									continue;//

								CollisionType collision_type=STATIC_STATIC;
								PhysPolygon *A1=0, *B1=0;
								bool swapped=false;
								if(A.mech_unstable=='S')//stable
								{
										 if(B.mech_unstable=='S')	collision_type=STATIC_STATIC,	swapped=false,	A1=&A, B1=&B;//static vs static stuck
									else if(B.mech_unstable=='U')	collision_type=STATIC_DYNAMIC,	swapped=false,	A1=&A, B1=&B;//static vs dynamic
									else							collision_type=STATIC_HINGE,	swapped=false,	A1=&A, B1=&B;//static vs hinge
								}
								else if(A.mech_unstable=='U')//unstable
								{
										 if(B.mech_unstable=='S')	collision_type=STATIC_DYNAMIC,	swapped=true,	A1=&B, B1=&A;//dynamic vs static
									else if(B.mech_unstable=='U')	collision_type=DYNAMIC_DYNAMIC,	swapped=false,	A1=&A, B1=&B;//dynamic vs dynamic
									else							collision_type=HINGE_DYNAMIC,	swapped=true,	A1=&B, B1=&A;//dynamic vs hinge
								}
								else//touching sth below
								{
										 if(B.mech_unstable=='S')	collision_type=STATIC_HINGE,	swapped=true,	A1=&B, B1=&A;//hinge vs static
									else if(B.mech_unstable=='U')	collision_type=HINGE_DYNAMIC,	swapped=false,	A1=&A, B1=&B;//hinge vs dynamic
									else							collision_type=HINGE_HINGE,		swapped=false,	A1=&A, B1=&B;//hinge vs hinge
								}

								PhysPolygon *A2, *B2;
								bool swapped_again=As_side==swapped;
								if(swapped_again)
									A2=B1, B2=A1;
								else
									A2=A1, B2=B1;
								//bool swapped_again=!As_side;
								//if(As_side!=swapped)
								//	A2=A1, B2=B1;
								//else
								//	A2=B1, B2=A1;
							/*	if(swapped)
								{
									if(As_side)
										A2=B1, B2=A1, swapped_again=false;
									else
										A2=A1, B2=B1, swapped_again=true;
								}
								else
								{
									if(As_side)
										A2=A1, B2=B1, swapped_again=false;
									else
										A2=B1, B2=A1, swapped_again=true;
								}//*/
								
								double &cx=B2->w[point].x, &cy=B2->w[point].y;
								int side_v2=(side+1)%A2->w.size();
								double
									&lx1=A2->w[side].x, &lx2=A2->w[side_v2].x,//collision line, far point, collision point
									&ly1=A2->w[side].y, &ly2=A2->w[side_v2].y;
								
								double nx=ly2-ly1, ny=-(lx2-lx1);//collision surface normal (pointing out of A2)
								if(info)
									draw_rectangle(cx-5, cx+5, cy-5, cy+5);
								bool A2_below_B2=ny>0,
									sfo=swapped!=swapped_again;//swapped from original
								
							//	A1->mech_unstable='U';
							//	B1->mech_unstable='U';
								if(A2_below_B2)
									B2->collisions.push_back(CollisionInfo(sfo?kb:ka, sfo!=As_side, point, side, travel, rotational, nx, ny));
								else
									A2->collisions.push_back(CollisionInfo(sfo?ka:kb, sfo==As_side, point, side, travel, rotational, nx, ny));
								//if(A2_below_B2)
								//{
								//	if(sfo)
								//	{
								//		if(As_side)	B2->collisions.rbegin()->others_size=false;
								//		else		B2->collisions.rbegin()->others_side=true;
								//	}
								//	else
								//	{
								//		if(As_side)	B2->collisions.rbegin()->others_size=true;
								//		else		B2->collisions.rbegin()->others_side=false;
								//	}
								//}
								//else
								//{
								//	if(sfo)
								//	{
								//		if(As_side)	A2->collisions.rbegin()->others_size=true;
								//		else		A2->collisions.rbegin()->others_side=false;
								//	}
								//	else
								//	{
								//		if(As_side)	A2->collisions.rbegin()->others_size=false;
								//		else		A2->collisions.rbegin()->others_side=true;
								//	}
								//}
							/*	switch(collision_type)
								{
								case STATIC_STATIC:
									//if(A2_below_B2)
									//	B2->resting_on.push_back(CollisionInfo(sfo?ka:kb, sfo!=As_side, point, side, travel, rotational, nx, ny));
									//else
									//	A2->resting_on.push_back(CollisionInfo(sfo?kb:ka, sfo==As_side, point, side, travel, rotational, nx, ny));
									break;
								case STATIC_DYNAMIC:
									A1->mech_unstable='U';
									if(A2_below_B2)
										B2->collisions.push_back(CollisionInfo(sfo?ka:kb, sfo!=As_side, point, side, travel, rotational, nx, ny));
									else
										A2->collisions.push_back(CollisionInfo(sfo?kb:ka, sfo==As_side, point, side, travel, rotational, nx, ny));
									break;
								case STATIC_HINGE:
									A1->mech_unstable='U';
									B1->mech_unstable='U';
									if(A2_below_B2)
										B2->collisions.push_back(CollisionInfo(sfo?ka:kb, sfo!=As_side, point, side, travel, rotational, nx, ny));
									else
										A2->collisions.push_back(CollisionInfo(sfo?kb:ka, sfo==As_side, point, side, travel, rotational, nx, ny));
									break;
								case DYNAMIC_DYNAMIC:
									if(A2_below_B2)
										B2->collisions.push_back(CollisionInfo(sfo?ka:kb, sfo!=As_side, point, side, travel, rotational, nx, ny));
									else
										A2->collisions.push_back(CollisionInfo(sfo?kb:ka, sfo==As_side, point, side, travel, rotational, nx, ny));
									break;
								case HINGE_DYNAMIC:
									A1->mech_unstable='U';
									if(A2_below_B2)
										B2->collisions.push_back(CollisionInfo(sfo?ka:kb, sfo!=As_side, point, side, travel, rotational, nx, ny));
									else
										A2->collisions.push_back(CollisionInfo(sfo?kb:ka, sfo==As_side, point, side, travel, rotational, nx, ny));
									break;
								case HINGE_HINGE:
									A1->mech_unstable='U';
									B1->mech_unstable='U';
									if(A2_below_B2)
										B2->collisions.push_back(CollisionInfo(sfo?ka:kb, sfo!=As_side, point, side, travel, rotational, nx, ny));
									else
										A2->collisions.push_back(CollisionInfo(sfo?kb:ka, sfo==As_side, point, side, travel, rotational, nx, ny));
									break;
								}//*/
							}
						}
					}
				}
			//	if(frame_number==143)
				//if(frame_number==221)
				//if(frame_number==676)
				//if(frame_number==306)
				//if(frame_number==35)
				//	int LOL_1=0;
				for(int k=0, kEnd=polygons.size();k<kEnd;++k)//detect collisions with boundaries
				{
					auto &A=polygons[k];
				//	if(A.mech_unstable!='S')
				//	{
						bool collision=false;
						if(A.p.y-A.r<yground)//ground
						{
							for(int kv=0, kvEnd=A.w.size();kv<kvEnd;++kv)
								if(A.w[kv].y<yground)
								{
									collision=true;
									break;
								}
							if(collision)
							{
								int point=0;	double t=A.w[0].y;
								for(int kv=1, kvEnd=A.w.size();kv<kvEnd;++kv)
									if(t>A.w[kv].y)
										point=kv, t=A.w[kv].y;
								int npoints=0;
								for(int kv=0, kvEnd=A.w.size();kv<kvEnd;++kv)
									npoints+=t==A.w[kv].y;
								if(A.mech_unstable=='S')//stuck in ground
									A.translate_world(Point(0, yground-A.w[point].y));
								else if(npoints>1)//face collision
								{
									if(A.mech_unstable=='U')
										A.collisions.push_back(CollisionInfo(-2, false, point, 0, yground-A.w[point].y, false, 0, 1));
									else if(A.mech_unstable=='G')
										A.resting_on.push_back(CollisionInfo(-2, false, point, 0, yground-A.w[point].y, false, 0, 1));
								}
								else
								{
									if(A.mech_unstable=='U')//dynamic (mechanically unstable)
										A.collisions.push_back(CollisionInfo(-2, true, point, 0, yground-A.w[point].y, false, 0, 1));
									else if(A.mech_unstable=='G')
										A.resting_on.push_back(CollisionInfo(-2, true, point, 0, yground-A.w[point].y, false, 0, 1));
								}
							}
						}
				//	}
					if(A.p.x-A.r<xleft)//left wall
					{
						bool collision=false;
						for(int kv=0, kvEnd=A.w.size();kv<kvEnd;++kv)
							if(A.w[kv].x<xleft)
							{
								collision=true;
								break;
							}
						if(collision)
						{
							int point=0;	double t=A.w[0].x;
							for(int kv=1, kvEnd=A.w.size();kv<kvEnd;++kv)
								if(t>A.w[kv].x)
									point=kv, t=A.w[kv].x;
							A.collisions.push_back(CollisionInfo(-2, true, point, 0, xleft-A.w[point].x, false, 1, 0));
							if(A.mech_unstable=='S')
								A.mech_unstable='U';
						}
					}
					if(A.p.x+A.r>xright)//right wall
					{
						bool collision=false;
						for(int kv=0, kvEnd=A.w.size();kv<kvEnd;++kv)
							if(A.w[kv].x>xright)
							{
								collision=true;
								break;
							}
						if(collision)
						{
							int point=0;	double t=A.w[0].x;
							for(int kv=1, kvEnd=A.w.size();kv<kvEnd;++kv)
								if(t<A.w[kv].x)
									point=kv, t=A.w[kv].x;
							A.collisions.push_back(CollisionInfo(-2, true, point, 0, A.w[point].x-xright, false, -1, 0));
							if(A.mech_unstable=='S')
								A.mech_unstable='U';
						}
					}
					if(A.p.y+A.r>ytop)//ceiling
					{
						bool collision=false;
						for(int kv=0, kvEnd=A.w.size();kv<kvEnd;++kv)
							if(A.w[kv].y>ytop)
							{
								collision=true;
								break;
							}
						if(collision)
						{
							int point=0;	double t=A.w[0].y;
							for(int kv=1, kvEnd=A.w.size();kv<kvEnd;++kv)
								if(t<A.w[kv].y)
									point=kv, t=A.w[kv].y;
							A.collisions.push_back(CollisionInfo(-2, true, point, 0, A.w[point].y-ytop, false, 0, -1));
							if(A.mech_unstable=='S')
								A.mech_unstable='U';
						}
					}
					if(A.mech_unstable!='S'&&!A.resting_on.size())
						A.mech_unstable='U';
				}
				//if(frame_number==36&&k==8)
				//if(frame_number==48)
				//if(frame_number==236)
				//if(frame_number==69)
				//if(frame_number==242)
				//if(frame_number==64)
				//if(scenario_number==2&&frame_number==109)
				//if(frame_number==306)
				//if(scenario_number==1&&frame_number==86)
				//if(scenario_number==2&&frame_number==107)
				//if(frame_number==177)
				//if(scenario_number==1&&frame_number==63)
				//if(frame_number==53)
				//if(frame_number==95)
				//if(frame_number==96)
				//if(frame_number==10)
				if(scenario_number==1&&frame_number==74)
					int LOL_1=0;
				double total_energy=0;
				for(int k=0, kEnd=polygons.size();k<kEnd;++k)//resolve & draw
				{
					//if(frame_number==221&&k==9)
					//if(frame_number==676&&k==7)
					//if(frame_number==346&&k==4)
					//if(scenario_number==2&&frame_number==109&&k==3)
					//if(scenario_number==1&&frame_number==83&&k==3)
					//if(frame_number==201&&k==4)
					if(frame_number==17)
						int LOL_1=0;
					auto &A=polygons[k];

					std::vector<int> q;//resolve dynamic collisions
					bool free_fall=A.mech_unstable!='S';
					for(int k2=0, k2End=A.collisions.size();k2<k2End;++k2)
					{
						auto &c=A.collisions[k2];
						switch(c.idx)
						{
						case -2://boundary
						//	free_fall&=!(c.nx==0&&c.ny==1);
							if(c.ny>0.707)//ground the body can rest on
							{
								free_fall=false;
								if(resolve_collision_polygon_vs_wall(A, c.point, c.nx, c.ny, c.travel, c.others_side))
								{
									A.mech_unstable='G';
									c.travel=0;
									A.resting_on.push_back(c);
								}
							}
							else
								resolve_collision_polygon_vs_wall(A, c.point, c.nx, c.ny, c.travel, c.others_side);
							break;
						case -1://nothing
							break;
						default://body
							if(resolve_collision_polygon_vs_polygon(A, polygons[c.idx], !c.others_side, c.point, c.side, c.travel, c.rotational))
							{
								A.mech_unstable='G';
								c.travel=0;
								A.resting_on.push_back(c);
							}
							bool problem=c.idx==k;
							for(int k3=0, k3End=q.size();k3<k3End;++k3)
								if(c.idx==q[k3])
								{
									problem=true;
									break;
								}
							if(!problem)
								q.push_back(c.idx);
							break;
						}
					}
					if(free_fall)
						A.v.y-=gravity*timescale;
					for(int wd=0, wdEnd=polygons.size();wd<wdEnd&&q.size();++wd)
					{
						int idx=q[0];
						auto &B=polygons[idx];
						for(int k2=0, k2End=B.collisions.size();k2<k2End;++k2)
						{
							auto &c=B.collisions[k2];
							free_fall=true;
							switch(c.idx)
							{
							case -2://boundary
								if(c.ny>0.707)//ground the body can rest on
								{
									free_fall=false;
									if(resolve_collision_polygon_vs_wall(B, c.point, c.nx, c.ny, c.travel, c.others_side))
									{
										B.mech_unstable='G';
										c.travel=0;
										B.resting_on.push_back(c);
									}
								}
								else
									resolve_collision_polygon_vs_wall(B, c.point, c.nx, c.ny, c.travel, c.others_side);
							//	free_fall&=!(c.nx==0&&c.ny==1);
								break;
							case -1://nothing
								break;
							default://body
								resolve_collision_polygon_vs_polygon(B, polygons[c.idx], !c.others_side, c.point, c.side, c.travel, c.rotational);
								bool problem=c.idx==idx;
								for(int k3=0, k3End=q.size();k3<k3End;++k3)
									if(c.idx==q[k3])
									{
										problem=true;
										break;
									}
								if(!problem)
									q.push_back(c.idx);
								break;
							}
							if(free_fall)
								A.v.y-=gravity*timescale;
						}
						B.collisions.resize(0);
						q.erase(q.begin());
					}
					A.collisions.resize(0);
					
					//if(frame_number==85&&k==4)
					//if(scenario_number==2&&frame_number==109&&k==3)
					//if(frame_number==201&&k==4)
					//if(frame_number==216&&k==3)
					//if(frame_number==107&&k==0)
					//if(scenario_number==1&&frame_number==107&&k==2)
					//	int LOL_1=0;
					for(int k2=0, k2End=A.resting_on.size();k2<k2End;++k2)		//resolve touch
				//	if(A.resting_on.size()==1)
					{
						auto &c=A.resting_on[k2];
						switch(c.idx)
						{
						case -2://boundary
							{
								Point d(c.travel*c.nx, c.travel*c.ny);
								A.translate_world(d);
								resolve_collision_lever(A, A.w[c.point]+d, c);
							}
							break;
						case -1://nothing
							break;
						default://object
							{
								auto &B=polygons[c.idx];
								if(c.others_side)//B's side, A's hinge
								{
								Point d(c.travel*c.nx, c.travel*c.ny);
									A.translate_world(d);//n pointing out of B
									resolve_collision_lever(A, A.w[c.point]+d, c);
								}
								else//A's side, B's hinge
								{
								Point point_d(c.travel*c.nx, c.travel*c.ny);
									A.translate_world(-point_d);//n pointing out of A
									resolve_collision_lever(A, B.w[c.point]+point_d, c);
								}
							}
							break;
						}
					/*	switch(c.idx)
						{
						case -2://boundary
							{
								A.vr+=A.mass*gravity*(A.w[c.point].x-A.p.x)*A.inv_J*timescale;
								A.rotate_around(A.w[c.point], A.vr*timescale);

								int point=0;	double t=A.w[0].y;//stand on ground
								for(int kv=1, kvEnd=A.w.size();kv<kvEnd;++kv)
									if(t>A.w[kv].y)
										point=kv, t=A.w[kv].y;
								if(A.w[point].y<yground)
									A.translate_world(Point(0, yground-A.w[point].y));
							//	A.translate_world(Point(0, yground-A.w[c.point].y));

								for(int k2=1, k2End=A.w.size();k2<k2End;++k2)
								{
									int kv=(c.point+k2)%A.w.size();
									if(A.w[kv].y<0)
									{
										Point v2=A.w[kv]-A.w[c.point];
										double angle_change=v2.angle();
										A.rotate_around(A.w[c.point], -angle_change);

										A.vr=-A.vr*damping;
										if(A.vr<0.1)
										{
											A.v.x=0, A.v.y=0;//
											A.vr=0;
											A.mech_unstable='S';
										}
										break;
									}
								}
							}
							break;
						case -1://nothing
							break;
						default://body
							{
								auto &B=polygons[c.idx];
								if(c.others_side)
								{
									A.vr+=A.mass*gravity*(A.w[c.point].x-A.p.x)*A.inv_J*timescale;//traction
									A.rotate_around(A.w[c.point], A.vr*timescale);

									int side_v2=(c.side+1)%B.w.size();//A stands on B
									double new_y=interpolation_line_nonvertical_y(B.w[c.side], B.w[side_v2], A.w[c.point].x);//-321
									Point min_y, max_y;
									if(B.w[c.side].y<B.w[side_v2].y)
										min_y=B.w[c.side], max_y=B.w[side_v2];
									else
										min_y=B.w[side_v2], max_y=B.w[c.side];
									if(new_y<min_y.y)
										A.translate_world(min_y-A.w[c.point]);
									else if(new_y>max_y.y)
										A.translate_world(max_y-A.w[c.point]);
									else
										A.translate_world(Point(0, new_y-A.w[c.point].y));
										
									for(int k2=1, k2End=A.w.size();k2<k2End;++k2)
									{
										int kv=(c.point+k2)%A.w.size();
										if(A.w[kv].y<yground)//rotate back until A.w[(point+k2)%s].y==0: sin(angle)=A.w[point].y/d2
										{
											double dx=A.w[kv].x-A.w[c.point].x, dy=A.w[kv].y-A.w[c.point].y;
											double d2=sqrt(dx*dx+dy*dy);
											int sign=A.p.x<A.w[c.point].x;
											sign-=!sign;

											if(dx<0&&dy<0)
												dx=-dx, dy=-dy;
											double sin_a=sign*(A.w[c.point].y-yground)/d2, cos_a=sqrt(1-sin_a*sin_a);
											A.rotate_around(A.w[c.point], (cos_a*dx+sin_a*dy)/d2, (sin_a*dx-cos_a*dy)/d2);

											A.vr=-A.vr*damping;
											if(A.vr<0.1)
											{
												A.v.x=0, A.v.y=0;//
												A.vr=0;
												A.mech_unstable='S';
											}
											break;
										}
									}
										
									double cos_ch=0, sin_ch=0;//angle change
								//	double angle_change=0;
									if(inclusion_triangle_point(A, B.w[(c.side+1)%B.w.size()]))//A's vertex resting on B's side		lever
									{//rotate until not intersecting
										double dx=B.w[(c.side+1)%B.w.size()].x-A.w[c.point].x, dy=B.w[(c.side+1)%B.w.size()].y-A.w[c.point].y, d=sqrt(dx*dx+dy*dy);
										if(d>1e-10)
										{
											double dx2=A.w[(c.point+2)%A.w.size()].x-A.w[c.point].x, dy2=A.w[(c.point+2)%A.w.size()].y-A.w[c.point].y, d2=sqrt(dx2*dx2+dy2*dy2);
											cos_ch=(dx*dx2+dy*dy2)/(d*d2), sin_ch=(dy*dx2-dy2*dx)/(d*d2);
										}
									}
									if(inclusion_triangle_point(A, B.w[c.side]))//A's vertex resting on B's side		lever
									{//rotate until not intersecting
										double dx=B.w[c.side].x-A.w[c.point].x, dy=B.w[c.side].y-A.w[c.point].y, d=sqrt(dx*dx+dy*dy);
										if(d>1e-10)
										{
											double dx2=A.w[(c.point+1)%A.w.size()].x-A.w[c.point].x, dy2=A.w[(c.point+1)%A.w.size()].y-A.w[c.point].y, d2=sqrt(dx2*dx2+dy2*dy2);
											cos_ch=(dx*dx2+dy*dy2)/(d*d2), sin_ch=(dy*dx2-dy2*dx)/(d*d2);
										}
									}
									if(cos_ch||sin_ch)
									{
										A.rotate_around(A.w[c.point], cos_ch, sin_ch);

										A.vr=-A.vr*damping;
										if(A.vr<0.1)
										{
											A.v.x=0, A.v.y=0;//
											A.vr=0;
											A.mech_unstable='S';
										}
									}
								}
								else
								{
									A.vr+=A.mass*gravity*(B.w[c.point].x-A.p.x)*A.inv_J*timescale;
									A.rotate_around(B.w[c.point], A.vr*timescale);
										
									double old_y=interpolation_line_nonvertical_y(A.w[c.side], A.w[(c.side+1)%A.w.size()], B.w[c.point].x);
									A.translate_world(Point(0, B.w[c.point].y-old_y));//A stands on B
										
									int lowpoint=A.w[c.side].y<A.w[(c.side+1)%A.w.size()].y ? c.side : (c.side+1)%3 ;
									if(A.w[lowpoint].y<0)//rotate back until A.w[lowpoint].y==0: sin(angle)=B.w[point].y/d2
									{
										double dx=A.w[lowpoint].x-B.w[c.point].x, dy=A.w[lowpoint].y-B.w[c.point].y;
										double d2=sqrt(dx*dx+dy*dy);
										int sign=A.p.x<A.w[lowpoint].x;
										sign-=!sign;
										if(dx<0&&dy<0)
											dx=-dx, dy=-dy;
										double sin_a=sign*B.w[c.point].y/d2, cos_a=sqrt(1-sin_a*sin_a);
										A.rotate_around(B.w[c.point], (cos_a*dx+sin_a*dy)/d2, (sin_a*dx-cos_a*dy)/d2);
											
										A.vr=-A.vr*damping;
										if(A.vr<0.1)
										{
											A.v.x=0, A.v.y=0;//
											A.vr=0;
											A.mech_unstable='S';
										}
									}
									double cos_ch=0, sin_ch=0;
									if(inclusion_triangle_point(B, A.w[c.side]))//A's side rests on B's vertex		see-saw
									{//rotate until not intersecting
										double dx=B.w[(c.point+1)%3].x-B.w[c.point].x, dy=B.w[(c.point+1)%3].y-B.w[c.point].y, d=sqrt(dx*dx+dy*dy);
										double dx2=A.w[c.side].x-B.w[c.point].x, dy2=A.w[c.side].y-B.w[c.point].y, d2=sqrt(dx2*dx2+dy2*dy2);
										if(d2>1e-10)
											cos_ch=(dx*dx2+dy*dy2)/(d*d2), sin_ch=(dy*dx2-dy2*dx)/(d*d2);
									}
									if(inclusion_triangle_point(B, A.w[(c.side+1)%A.w.size()]))//A's side rests on B's vertex		see-saw
									{//rotate until not intersecting
										double dx=B.w[(c.point+2)%B.w.size()].x-B.w[c.point].x, dy=B.w[(c.point+2)%B.w.size()].y-B.w[c.point].y, d=sqrt(dx*dx+dy*dy);
										double dx2=A.w[(c.side+1)%3].x-B.w[c.point].x, dy2=A.w[(c.side+1)%3].y-B.w[c.point].y, d2=sqrt(dx2*dx2+dy2*dy2);
										if(d2>1e-10)
											cos_ch=(dx*dx2+dy*dy2)/(d*d2), sin_ch=(dy*dx2-dy2*dx)/(d*d2);
									}
									if(cos_ch||sin_ch)
									{
										A.rotate_around(B.w[c.point], cos_ch, sin_ch);

										A.vr=-A.vr*damping;
										if(A.vr<0.1)
										{
											A.v.x=0, A.v.y=0;//
											A.vr=0;
											A.mech_unstable='S';
										}
									}
								}
							}
							break;
						}//*/
					}
					A.resting_on.resize(0);

					if(info)
					{
						double kinetic_energy=0.5*A.mass*A.v.mag_sq()+0.5*A.J*A.vr*A.vr, potential_energy=A.mass*gravity*(A.p.y-yground);
						total_energy+=kinetic_energy+potential_energy;
						GUIPrint(ghMemDC, 0, 18+18*k, "%d: %c, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf", k, A.mech_unstable, A.v.x, A.v.y, A.vr, A.p.x, A.p.y, kinetic_energy, potential_energy);
						draw_line(A.p.x, A.p.y, A.w[0].x, A.w[0].y);
						GUIPrint(ghMemDC, A.p.x-4, A.p.y+9, "%d", k);

						//A.iterate(timescale);
						//HPEN hPen=CreatePen(PS_DOT, 1, 0xFF0000);
						//hPen=(HPEN)SelectObject(ghMemDC, hPen);
						//A.draw();
						//hPen=(HPEN)SelectObject(ghMemDC, hPen);
						//DeleteObject(hPen);
						//A.iterate(-timescale);
					}
					A.draw();
				}
				//for(int k=0, kEnd=polygons.size();k<kEnd;++k)//
				//{
				//	auto &A=polygons[k];
				//	if(A.collisions.size()||A.resting_on.size())//
				//		int LOL_1=0;
				//}
#endif
				GUIPrint(ghMemDC, w*2/5, 36, "%d contacts, %d points", contacts.size(), npoints);
				//GUIPrint(ghMemDC, w*2/5, 36, "contacts: %d", contacts.size());
				if(!warm_starting)
					contacts.resize(0);
#endif
				++frame_number;
				tick=false;
			}
			double total_energy=0;
			for(int k=0, kEnd=polygons.size();k<kEnd;++k)
			{
				auto &A=polygons[k];
				double kinetic_energy=0.5*A.mass*A.v.mag_sq()+0.5*A.J*A.vr*A.vr, potential_energy=A.mass*gravity*(A.p.y-yground);
				total_energy+=kinetic_energy+potential_energy;
				if(info)
				{
					GUIPrint(ghMemDC, 0, 18+18*k, "%d: %c, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf", k, A.mech_unstable, A.v.x, A.v.y, A.vr, A.p.x, A.p.y, kinetic_energy, potential_energy);
				//	GUIPrint(ghMemDC, 0, 18+18*k, "%d: %c, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %d", k, A.mech_unstable, A.v.x, A.v.y, A.vr, A.p.x, A.p.y, kinetic_energy, potential_energy, A.collisions.size());
					draw_line(A.p.x, A.p.y, A.w[0].x, A.w[0].y);
					GUIPrint(ghMemDC, A.p.x-4, A.p.y+9, "%d", k);

					//A.iterate(timescale);
					//HPEN hPen=CreatePen(PS_DOT, 1, 0xFF0000);
					//hPen=(HPEN)SelectObject(ghMemDC, hPen);
					//A.draw();
					//hPen=(HPEN)SelectObject(ghMemDC, hPen);
					//DeleteObject(hPen);
					//A.iterate(-timescale);
				}
				A.draw();
			}
			if(info)
				GUIPrint(ghMemDC, 0, 0, "idx, mech_unstable, vx, vy, vr, px, py, KE, PE");
			GUIPrint(ghMemDC, w*2/5, 0, "frame #%08d, scenario #%03d", frame_number-1, scenario_number);
			GUIPrint(ghMemDC, w*2/5, 18, "total energy: %lf", total_energy);
			draw_rectangle(xleft, xright, yground, ytop);
			GUIPrint(ghMemDC, w-300, 0, "P, F: pause");
			GUIPrint(ghMemDC, w-300, 18, "R: reset unpaused");
			GUIPrint(ghMemDC, w-300, 18*2, "T: reset paused");
			GUIPrint(ghMemDC, w-300, 18*3, "ENTER, G: next frame");
			GUIPrint(ghMemDC, w-300, 18*4, "Y: toggle info");
			GUIPrint(ghMemDC, w-300, 18*5, "O: RH pause (%s)", rotational_pause?"on":"off");
			GUIPrint(ghMemDC, w-300, 18*6, "E: stop controlled triangle 0");
			GUIPrint(ghMemDC, w-300, 18*7, "4: new scenario");
			GUIPrint(ghMemDC, w-300, 18*8, "5: new scenario paused");

			BitBlt(ghDC, 0, 0, w, h, ghMemDC, 0, 0, SRCCOPY);
		}
		break;
	case WM_TIMER:
		if(polygons.size())
		{
			auto &A=polygons[0];
			if(kb[VK_LEFT])
			{
				if(-A.v.x<runLimit)
					A.v.x-=runForce*timescale;
			}
			else if(kb[VK_RIGHT])
			{
				if(A.v.x<runLimit)
					A.v.x+=runForce*timescale;
			}
			if((kb[' ']||kb[VK_UP])&&(clock()-jumpStart<jumpDuration))
				A.v.y=jumpVelocity;
			if(kb['E'])
			{
				if(kb[VK_SHIFT])
				{
					auto &A=polygons[0];
					A.v.x=A.v.y=A.vr=0;
				}
				else
				{
					for(int k=0, kEnd=polygons.size();k<kEnd;++k)
					{
						auto &A=polygons[k];
						A.v.x/=2, A.v.y/=2, A.vr/=2;
					//	A.vx=A.vy=A.vr=0;
					}
				}
			}
		}
		tick=true;
		InvalidateRect(hWnd, 0, false);
		return 0;
	case WM_SIZE:
		break;
	case WM_LBUTTONDOWN:
		break;
	case WM_RBUTTONDOWN:
		break;
	case WM_MOUSEMOVE:
		break;
	case WM_LBUTTONUP:
		break;
	case WM_RBUTTONUP:
		break;
	case WM_MOUSEWHEEL:
		break;
	case WM_KEYDOWN:
		kb[wParam]=true;
		switch(wParam)
		{
		case VK_RETURN:case 'G'://next frame
			if(!timer)
			{
				tick=true;
				InvalidateRect(hWnd, 0, false);
			}
		//	tick=true;
		//	InvalidateRect(hWnd, 0, false);
			break;
		case 'Y'://toggle info
			if(!timer)
			{
			//	tick=true;
				InvalidateRect(hWnd, 0, false);
			}
			info=!info;
			break;
	//	case 'E':
	//		break;
		case ' ':case VK_UP:
			if(!(lParam&0x40000000))
		//	if(!(lParam&0x40000000)&&!freeFall)
				jumpStart=clock();
			break;
		case 'P':case 'F'://toggle pause
			if(timer)
			{
				KillTimer(hWnd, 0);
				timer=false, pause=true;
			}
			else
			{
				timer=true, pause=false;
				SetTimer(hWnd, 0, 10, 0);
			}
			break;
		case 'O':
			rotational_pause=!rotational_pause;
			break;
		case 'T'://reset paused
			lastlambda=lastlambda_walls=0;//
			frame_number=0;
			polygons=polygons0;
			for(int k=0, kEnd=polygons.size();k<kEnd;++k)
				polygons[k].iterate(0);
			contacts.resize(0);
		//	initialize();
			if(timer)
			{
				KillTimer(hWnd, 0);
				timer=false, pause=true;
			}
			tick=true;
			InvalidateRect(hWnd, 0, false);
			break;
		case '5'://new scenario paused
			lastlambda=lastlambda_walls=0;//
			frame_number=0;
			initialize(), ++scenario_number;
			if(timer)
			{
				KillTimer(hWnd, 0);
				timer=false, pause=true;
			}
			tick=true;
			InvalidateRect(hWnd, 0, false);
			break;
		case 'R'://reset
			lastlambda=lastlambda_walls=0;//
			frame_number=0;
			polygons=polygons0;
			for(int k=0, kEnd=polygons.size();k<kEnd;++k)
				polygons[k].iterate(0);
			contacts.resize(0);
		//	initialize();
			if(pause)//
			{
				timer=true, pause=false;
				SetTimer(hWnd, 0, 10, 0);
			}
			if(!timer)
			{
				tick=true;
				InvalidateRect(hWnd, 0, false);
			}
			break;
		case '4'://new scenario
			lastlambda=lastlambda_walls=0;//
			frame_number=0;
			initialize(), ++scenario_number;
			if(pause)//
			{
				timer=true, pause=false;
				SetTimer(hWnd, 0, 10, 0);
			}
			if(!timer)
			{
				tick=true;
				InvalidateRect(hWnd, 0, false);
			}
			break;
		case 'X':PostQuitMessage(0);
			return 0;
		}
		break;
	case WM_KEYUP:
		kb[wParam]=false;
		break;
	case WM_CLOSE:PostQuitMessage(0);
		return 0;
	}
	return DefWindowProcA(hWnd, message, wParam, lParam);
}
int __stdcall	WinMain(HINSTANCE hInstance, HINSTANCE, char*, int nCmdShow)
{
	WNDCLASSEXA wndClassEx={sizeof(WNDCLASSEXA), CS_HREDRAW|CS_VREDRAW|CS_DBLCLKS, WndProc, 0, 0, hInstance, LoadIconA(0, (char*)0x00007F00), LoadCursorA(0, (char*)0x00007F00), (HBRUSH__*)(COLOR_WINDOW+1), 0, "New format", 0};
	RegisterClassExA(&wndClassEx);
	ghWnd=CreateWindowExA(0, wndClassEx.lpszClassName, "Physics 2 2016-08-15Mo", WS_CAPTION|WS_SYSMENU|WS_THICKFRAME|WS_MINIMIZEBOX|WS_MAXIMIZEBOX|WS_CLIPCHILDREN, CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, 0, 0, hInstance, 0);
	ShowWindow(ghWnd, nCmdShow);
	tagMSG msg;
	for(;GetMessageA(&msg, 0, 0, 0);)TranslateMessage(&msg), DispatchMessageA(&msg);
	return msg.wParam;
}