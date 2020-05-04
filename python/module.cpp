
#include "module.h"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>



PYBIND11_MODULE(ctrigrid_bindings, m) 
{
    pybind11::class_<CTRIGRID_Vector3_wrapper>(m, "vec3")
        .def(pybind11::init<float, float, float>(),
            pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"))
        .def_readwrite("x", &CTRIGRID_Vector3_wrapper::x)
        .def_readwrite("y", &CTRIGRID_Vector3_wrapper::y)
        .def_readwrite("z", &CTRIGRID_Vector3_wrapper::z);

    pybind11::class_<CTRIGRID_UniformGrid_wrapper, std::unique_ptr<CTRIGRID_UniformGrid_wrapper>>(m, "grid")
        .def(pybind11::init<uint32_t, uint32_t, uint32_t, float, const CTRIGRID_Vector3_wrapper&>(),
            pybind11::arg("nx"), pybind11::arg("ny"), pybind11::arg("nz"), pybind11::arg("cell_width"), pybind11::arg("origin"))
        .def("closest_point", &CTRIGRID_UniformGrid_wrapper::FindClosestPointOnTris,
            pybind11::arg("p"))
        .def("closest_points", &CTRIGRID_UniformGrid_wrapper::FindAllClosestPointsOnTris,
            pybind11::arg("points"), pybind11::arg("force_in_grid") = false)
        .def("add_tri_mesh", &CTRIGRID_UniformGrid_wrapper::AddTris, 
            pybind11::arg("vertices"), pybind11::arg("indices"))
        .def("mem_footprint", &CTRIGRID_UniformGrid_wrapper::EstimateNativeMemory, 
            "Estimated footprint of grid in memory (native heap only)");

    m.def("closest_point_tri", &CTRIGRID_ComputeClosestPointOnTri, 
        "Returns the vec3 that lies on triangle (v0, v1, v2) and closest to p",
        pybind11::arg("v0"), pybind11::arg("v1"), 
        pybind11::arg("v2"), pybind11::arg("p"));

    m.def("compute_vertex_normals", &CTRIGRID_ComputeVertexNormals, 
        "Computes the (normalized) per vertex normals of the input triangle mesh",
        pybind11::arg("vertices"), pybind11::arg("indices"));
    m.def("compute_tri_normals", &CTRIGRID_ComputeTriNormals, 
        "Computes the (normalized) per tri normals of the input triangle mesh",
        pybind11::arg("vertices"), pybind11::arg("indices"));

    m.def("unique_edges", &CTRIGRID_GetUniqueEdges, "...", //pybind11::return_value_policy::copy,
        pybind11::arg("indices"));

    m.def("load_obj", &CTRIGRID_LoadObjFile, "Load vertex and index data from OBJ file",
        pybind11::arg("filename"));
    m.def("save_obj", &CTRIGRID_SaveObjFile, "Save vertex and index data to OBJ file",
        pybind11::arg("filename"), pybind11::arg("vertices"), pybind11::arg("indices"));
}

