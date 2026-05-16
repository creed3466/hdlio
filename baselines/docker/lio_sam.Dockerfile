# lio_sam.Dockerfile — LIO-SAM (brytsknguyen/LIO-SAM, NTU VIRAL canonical fork)
# Extra deps: GTSAM 4.0.3
# Target: NTU VIRAL only (9-axis IMU requirement).
#
# Prebuilt variant: LIO-SAM catkin workspace is compiled at image-build time
# (catkin_make -j2 with full host RAM available to a single build process).
# Runtime containers only copy the per-seq config over, launch nodes, and
# record odometry — no parallel compile → no cc1plus OOM kills.

FROM baselines-base:ros1

ENV CATKIN_WS=/root/catkin_ws
ENV GTSAM_VERSION=4.0.3
ENV LIOSAM_DIR=/root/catkin_ws/src/LIO-SAM

# ---------- GTSAM 4.0.3 (LIO-SAM requirement) ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
        libboost-all-dev libtbb-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch ${GTSAM_VERSION} https://github.com/borglab/gtsam.git /tmp/gtsam && \
    cd /tmp/gtsam && mkdir build && cd build && \
    cmake -DGTSAM_USE_SYSTEM_EIGEN=ON \
          -DGTSAM_BUILD_TESTS=OFF \
          -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
          -DGTSAM_WITH_TBB=OFF \
          .. && \
    make -j"$(nproc)" install && \
    ldconfig /usr/local/lib && \
    rm -rf /tmp/gtsam

# livox driver (required by LIO-SAM CMake even if not used for NTU)
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git ${CATKIN_WS}/src/livox_ros_driver

# ---------- FLANN patch (PCL 1.10 on Noetic compat) ----------
# PCL 1.10's FLANN LSH serializer calls std::unordered_map::serialize which
# does not exist. Inject a Serializer specialization.
RUN python3 - <<'PY'
hdr = '/usr/include/flann/util/serialization.h'
with open(hdr) as f:
    src = f.read()
if '// --- BEGIN LIO-SAM Noetic unordered_map patch ---' in src:
    raise SystemExit(0)
insert = r"""
// --- BEGIN LIO-SAM Noetic unordered_map patch ---
#include <unordered_map>
template<typename K, typename V>
struct Serializer<std::unordered_map<K, std::vector<V> > >
{
    template<typename InputArchive>
    static inline void load(InputArchive& ar, std::unordered_map<K, std::vector<V> >& m) {
        size_t n; ar & n;
        m.clear();
        for (size_t i = 0; i < n; ++i) {
            K k; ar & k;
            size_t sz; ar & sz;
            std::vector<V> v(sz);
            for (size_t j = 0; j < sz; ++j) ar & v[j];
            m.emplace(std::move(k), std::move(v));
        }
    }
    template<typename OutputArchive>
    static inline void save(OutputArchive& ar, const std::unordered_map<K, std::vector<V> >& m) {
        size_t n = m.size(); ar & n;
        for (const auto& kv : m) {
            K k = kv.first; ar & k;
            size_t sz = kv.second.size(); ar & sz;
            for (size_t j = 0; j < sz; ++j) ar & const_cast<V&>(kv.second[j]);
        }
    }
};
// --- END LIO-SAM Noetic unordered_map patch ---

"""
marker = '// declare serializers for simple types'
idx = src.find(marker)
if idx < 0:
    raise SystemExit('insertion marker not found in FLANN serialization header')
with open(hdr, 'w') as f:
    f.write(src[:idx] + insert + src[idx:])
print('[ok] patched', hdr)
PY

# ---------- Bake LIO-SAM source + build catkin workspace ----------
# Build context = baselines/; submodule checkout lives at algorithms/lio_sam/.
COPY algorithms/lio_sam ${LIOSAM_DIR}

# Install minimal headless run.launch (overwrites fork's original run.launch).
COPY docker/lio_sam_ntu_run.launch ${LIOSAM_DIR}/launch/run.launch

# catkin_make with -j2 — single build inside the image has full host RAM;
# -j2 caps peak footprint on mapOptmization.cpp (GTSAM-heavy TU).
RUN bash -c "source /opt/ros/noetic/setup.bash && \
             cd ${CATKIN_WS} && \
             catkin_make -j2 -DCMAKE_BUILD_TYPE=Release" && \
    ldconfig /usr/local/lib

# GTSAM 4.x installs libmetis-gtsam.so to /usr/local/lib; ensure runtime finds it.
ENV LD_LIBRARY_PATH=/usr/local/lib

WORKDIR ${CATKIN_WS}
CMD ["/bin/bash"]
