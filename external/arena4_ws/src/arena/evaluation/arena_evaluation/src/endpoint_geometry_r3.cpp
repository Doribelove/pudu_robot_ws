// Native translation of HospitalMap.footprint_collision for local connectors.
// The canonical Python PathAudit remains the final authority.
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

struct Point { double x, y; };
struct Checker {
  Py_buffer grid{};
  double resolution{}, ox{}, oy{};
  std::vector<Point> footprint;
  ~Checker() { if (grid.obj) PyBuffer_Release(&grid); }
};
constexpr const char * kCapsule = "arena.endpoint_geometry_r3.checker.v1";

static void destroy(PyObject * capsule) {
  delete static_cast<Checker *>(PyCapsule_GetPointer(capsule, kCapsule));
}

static PyObject * make_checker(PyObject *, PyObject * args) {
  PyObject * grid; PyObject * footprint; double resolution, ox, oy;
  if (!PyArg_ParseTuple(args, "OdddO", &grid, &resolution, &ox, &oy, &footprint)) return nullptr;
  auto * checker = new Checker();
  if (PyObject_GetBuffer(grid, &checker->grid, PyBUF_FORMAT | PyBUF_STRIDES) < 0) {
    delete checker; return nullptr;
  }
  if (checker->grid.ndim != 2 || checker->grid.itemsize != 1 ||
      !checker->grid.format || checker->grid.format[0] != 'b' ||
      !PyBuffer_IsContiguous(&checker->grid, 'C') || !std::isfinite(resolution) ||
      resolution <= 0 || !std::isfinite(ox) || !std::isfinite(oy)) {
    delete checker; PyErr_SetString(PyExc_ValueError, "Expected contiguous int8 grid and finite map geometry"); return nullptr;
  }
  PyObject * sequence = PySequence_Fast(footprint, "footprint must be a sequence");
  if (!sequence) { delete checker; return nullptr; }
  for (Py_ssize_t i = 0; i < PySequence_Fast_GET_SIZE(sequence); ++i) {
    PyObject * pair = PySequence_Fast(PySequence_Fast_GET_ITEM(sequence, i), "footprint point must be a pair");
    if (!pair) { Py_DECREF(sequence); delete checker; return nullptr; }
    if (PySequence_Fast_GET_SIZE(pair) != 2) {
      Py_DECREF(pair); Py_DECREF(sequence); delete checker;
      PyErr_SetString(PyExc_ValueError, "footprint point must have two coordinates"); return nullptr;
    }
    Point point{PyFloat_AsDouble(PySequence_Fast_GET_ITEM(pair, 0)), PyFloat_AsDouble(PySequence_Fast_GET_ITEM(pair, 1))};
    Py_DECREF(pair);
    if (PyErr_Occurred() || !std::isfinite(point.x) || !std::isfinite(point.y)) {
      Py_DECREF(sequence); delete checker;
      if (!PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, "nonfinite footprint");
      return nullptr;
    }
    checker->footprint.push_back(point);
  }
  Py_DECREF(sequence);
  if (checker->footprint.size() < 3) {
    delete checker; PyErr_SetString(PyExc_ValueError, "footprint needs at least three vertices"); return nullptr;
  }
  checker->resolution = resolution; checker->ox = ox; checker->oy = oy;
  return PyCapsule_New(checker, kCapsule, destroy);
}

static bool inside(Point p, const std::vector<Point> & polygon) {
  bool result = false;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto a = polygon[i], b = polygon[(i+1)%polygon.size()];
    if ((a.y > p.y) != (b.y > p.y) &&
        p.x < (b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x) result = !result;
  }
  return result;
}

static double segment_distance(Point p, Point a, Point b) {
  const double dx = b.x-a.x, dy = b.y-a.y, length2 = dx*dx+dy*dy;
  if (length2 <= 1e-12) return std::hypot(p.x-a.x, p.y-a.y);
  const double t = std::max(0., std::min(1., ((p.x-a.x)*dx+(p.y-a.y)*dy)/length2));
  return std::hypot(p.x-(a.x+t*dx), p.y-(a.y+t*dy));
}

static PyObject * collision(PyObject *, PyObject * args) {
  PyObject * capsule; double x, y, yaw, padding;
  if (!PyArg_ParseTuple(args, "Odddd", &capsule, &x, &y, &yaw, &padding)) return nullptr;
  auto * c = static_cast<Checker *>(PyCapsule_GetPointer(capsule, kCapsule));
  if (!c) return nullptr;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) || !std::isfinite(padding) || padding < 0) {
    PyErr_SetString(PyExc_ValueError, "nonfinite pose or negative padding"); return nullptr;
  }
  auto local = c->footprint;
  if (padding > 0) {
    double x0=local[0].x, x1=x0, y0=local[0].y, y1=y0;
    for (auto p : local) { x0=std::min(x0,p.x); x1=std::max(x1,p.x); y0=std::min(y0,p.y); y1=std::max(y1,p.y); }
    local={{x0-padding,y0-padding},{x0-padding,y1+padding},{x1+padding,y1+padding},{x1+padding,y0-padding}};
  }
  const double cs=std::cos(yaw), sn=std::sin(yaw), res=c->resolution;
  std::vector<Point> polygon; polygon.reserve(local.size());
  for (auto p : local) polygon.push_back({x+p.x*cs-p.y*sn,y+p.x*sn+p.y*cs});
  double xmin=polygon[0].x, xmax=xmin, ymin=polygon[0].y, ymax=ymin;
  for (auto p : polygon) { xmin=std::min(xmin,p.x); xmax=std::max(xmax,p.x); ymin=std::min(ymin,p.y); ymax=std::max(ymax,p.y); }
  const auto height=c->grid.shape[0], width=c->grid.shape[1];
  // Match the original predicate's one-cell expanded bounds and fail-closed map edge.
  const double c0d=std::floor((xmin-res-c->ox)/res), c1d=std::floor((xmax+res-c->ox)/res);
  const double r0d=height-1-std::floor((ymax+res-c->oy)/res), r1d=height-1-std::floor((ymin-res-c->oy)/res);
  if (c0d<0 || c1d>=width || r0d<0 || r1d>=height) Py_RETURN_TRUE;
  const auto * data=static_cast<const int8_t *>(c->grid.buf);
  const double half_diagonal=std::sqrt(2.)*res/2.;
  for (Py_ssize_t row=static_cast<Py_ssize_t>(r0d); row<=static_cast<Py_ssize_t>(r1d); ++row) {
    for (Py_ssize_t col=static_cast<Py_ssize_t>(c0d); col<=static_cast<Py_ssize_t>(c1d); ++col) {
      const auto value=data[row*width+col];
      if (value != 100 && value >= 0) continue;
      const Point center{c->ox+(col+.5)*res,c->oy+(height-row-.5)*res};
      if (inside(center,polygon)) Py_RETURN_TRUE;
      for (size_t i=0; i<polygon.size(); ++i) {
        if (segment_distance(center,polygon[i],polygon[(i+1)%polygon.size()])<=half_diagonal) Py_RETURN_TRUE;
      }
    }
  }
  Py_RETURN_FALSE;
}

// Union the exact horizontal runs of OpenCV's ellipse over a sparse raster.
// This is binary dilation, including clipping at the image boundary.
static PyObject * dilate_runs(PyObject *, PyObject * args) {
  PyObject * raster; PyObject * radii;
  if (!PyArg_ParseTuple(args,"OO",&raster,&radii)) return nullptr;
  Py_buffer input{};
  if (PyObject_GetBuffer(raster,&input,PyBUF_FORMAT|PyBUF_STRIDES)<0) return nullptr;
  if (input.ndim!=2 || input.itemsize!=1 || !input.format || input.format[0]!='B' ||
      !PyBuffer_IsContiguous(&input,'C')) {
    PyBuffer_Release(&input); PyErr_SetString(PyExc_ValueError,"expected contiguous uint8 raster"); return nullptr;
  }
  PyObject * seq=PySequence_Fast(radii,"kernel radii must be a sequence");
  if (!seq) { PyBuffer_Release(&input); return nullptr; }
  const auto size=PySequence_Fast_GET_SIZE(seq);
  std::vector<Py_ssize_t> widths;
  for (Py_ssize_t i=0;i<size;++i) {
    auto radius=PyLong_AsSsize_t(PySequence_Fast_GET_ITEM(seq,i));
    if (PyErr_Occurred() || radius<0 || radius>size) {
      Py_DECREF(seq); PyBuffer_Release(&input);
      if (!PyErr_Occurred()) PyErr_SetString(PyExc_ValueError,"invalid kernel radius");
      return nullptr;
    }
    widths.push_back(radius);
  }
  Py_DECREF(seq);
  if (size==0 || size%2==0) {
    PyBuffer_Release(&input); PyErr_SetString(PyExc_ValueError,"kernel needs odd positive height"); return nullptr;
  }
  auto * result=PyBytes_FromStringAndSize(nullptr,input.len);
  if (!result) { PyBuffer_Release(&input); return nullptr; }
  auto * output=PyBytes_AS_STRING(result); std::memset(output,0,input.len);
  const auto * data=static_cast<const uint8_t *>(input.buf);
  const auto h=input.shape[0],w=input.shape[1];
  for (Py_ssize_t y=0;y<h;++y) for (Py_ssize_t x=0;x<w;) {
    if (!data[y*w+x]) { ++x; continue; }
    auto first=x; while(x<w && data[y*w+x]) ++x;
    for (Py_ssize_t k=0;k<size;++k) {
      const auto row=y+k-size/2;
      if (row<0 || row>=h) continue;
      const auto left=std::max<Py_ssize_t>(0,first-widths[k]);
      const auto right=std::min(w,x+widths[k]);
      std::memset(output+row*w+left,1,right-left);
    }
  }
  PyBuffer_Release(&input); return result;
}

static PyMethodDef methods[] = {
  {"dilate_runs",dilate_runs,METH_VARARGS,"Exact binary symmetric run-kernel dilation."},
  {"make_checker",make_checker,METH_VARARGS,"Bind the original full footprint and int8 occupancy."},
  {"collision",collision,METH_VARARGS,"Original cell-center/polygon-distance predicate; unknown collides."},
  {nullptr,nullptr,0,nullptr}
};
static PyModuleDef module = {PyModuleDef_HEAD_INIT,"_endpoint_geometry_r3",nullptr,-1,methods,nullptr,nullptr,nullptr,nullptr};
PyMODINIT_FUNC PyInit__endpoint_geometry_r3() { return PyModule_Create(&module); }
