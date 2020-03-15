// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <assert.h>
#include <open_chisel/threading/Threading.h>
#include <open_chisel/ChunkManager.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/geometry/AABB.h>
#include <open_chisel/marching_cubes/MarchingCubes.h>
#include <open_chisel/geometry/Raycast.h>
#include <iostream>

namespace chisel
{

// 默认构造
ChunkManager::ChunkManager()
    : chunkSize(16, 16, 16), voxelResolutionMeters(0.03)
{
    CacheCentroids();
}

ChunkManager::~ChunkManager()
{}

// 构造函数
ChunkManager::ChunkManager(const Eigen::Vector3i &size, float res, bool color)
    : chunkSize(size), voxelResolutionMeters(res), useColor(color)
{
    // 体素中心
    CacheCentroids();
}

void ChunkManager::CacheCentroids()
{
    // half就是中心
    Vec3 halfResolution = Vec3(voxelResolutionMeters, voxelResolutionMeters, voxelResolutionMeters) * 0.5f;
    // chunk体素中心赋值
    centroids.resize(static_cast<size_t>(chunkSize(0) * chunkSize(1) * chunkSize(2)));
    int i = 0;
    for (int z = 0; z < chunkSize(2); z++)
    {
        for (int y = 0; y < chunkSize(1); y++)
        {
            for (int x = 0; x < chunkSize(0); x++)
            {
                centroids[i] = Vec3(x, y, z) * voxelResolutionMeters + halfResolution;
                i++;
            }
        }
    }

    // mesh用的，每一列是一个索引坐标偏移
    cubeIndexOffsets << 0, 1, 1, 0, 0, 1, 1, 0,
                        0, 0, 1, 1, 0, 0, 1, 1,
                        0, 0, 0, 0, 1, 1, 1, 1;
}

void ChunkManager::GetChunkIDsIntersecting(const AABB &box, ChunkIDList *chunkList)
{
    assert(chunkList != nullptr);

    ChunkID minID = GetIDAt(box.min);
    ChunkID maxID = GetIDAt(box.max) + Eigen::Vector3i(1, 1, 1);

    for (int x = minID(0); x < maxID(0); x++)
    {
        for (int y = minID(1); y < maxID(1); y++)
        {
            for (int z = minID(2); z < maxID(2); z++)
            {
                chunkList->push_back(ChunkID(x, y, z));
            }
        }
    }
}

void ChunkManager::RecomputeMesh(const ChunkID &chunkID, std::mutex &mutex)
{
    //mutex.lock();
    if (!HasChunk(chunkID))
    {
        //mutex.unlock();
        return;
    }

    MeshPtr mesh;
    if (!HasMesh(chunkID))//还没上锁，会执行么？
    {
        mesh = std::allocate_shared<Mesh>(Eigen::aligned_allocator<Mesh>());
    }
    else
    {
        mesh = GetMesh(chunkID);
    }

    ChunkPtr chunk = GetChunk(chunkID);
    //mutex.unlock();

    double t = clock();
    GenerateMesh(chunk, mesh.get());

    if (useColor)
    {
        ColorizeMesh(mesh.get());
    }

    ComputeNormalsFromGradients(mesh.get());

    mutex.lock();
    if (!mesh->grids.empty())
        //if (!mesh->vertices.empty())
        allMeshes[chunkID] = mesh;//更新chunk的mesh
    mutex.unlock();
}

void ChunkManager::RecomputeMeshes(const ChunkSet &chunkMeshes)
{

    if (chunkMeshes.empty())
    {
        return;
    }

    std::vector<ChunkID> chunkIDList;// 需更新的chunk
    for (const std::pair<ChunkID, bool> &chunk : chunkMeshes)
        if (chunk.second)
            chunkIDList.push_back(chunk.first);

    int nThread = 16;
    std::vector<std::thread> threads;
    std::mutex mutex;
    int n = chunkIDList.size();
    // 线程块大小
    int blockSize = (n + nThread - 1) / nThread;

    for (int i = 0; i < nThread; i++)
    {
        int s = i * blockSize;// 线程块首索引号
        printf("thread: %d, s: %d, blockSize: %d\n", i, s, blockSize);
        threads.push_back(std::thread([this, &mutex, &chunkIDList, n, s, blockSize]()
                                      {
                                        // 给锁了
                                        for (int j = 0, k = s + j; j < blockSize && k < n; j++, k++)
                                          RecomputeMesh(chunkIDList[k], mutex);
                                      }));
    }
    for (int i = 0; i < nThread; i++)
        threads[i].join();

    //for (const std::pair<ChunkID, bool> &chunk : chunkMeshes)
    ////parallel_for(chunks.begin(), chunks.end(), [this, &mutex](const std::pair<ChunkID, bool> &chunk)
    //{
    //    if (chunk.second)
    //        this->RecomputeMesh(ChunkID(chunk.first), mutex);
    //}
    ////);
}

/**
 * @brief 创建
 */
ChunkMap::iterator ChunkManager::CreateChunk(const ChunkID &id)
{
    return AddChunk(std::allocate_shared<Chunk>(Eigen::aligned_allocator<Chunk>(), id, chunkSize, voxelResolutionMeters, useColor));
}

void ChunkManager::Reset()
{
    allMeshes.clear();
    chunks.clear();
}

/**
 * @brief
 * @param frustum 当前视锥
 * @param chunkList chunk的列表vector<ChunkID>，ChunkID是三维的
 */
void ChunkManager::GetChunkIDsIntersecting(const Frustum &frustum, ChunkIDList *chunkList)
{
    assert(chunkList != nullptr);

    AABB frustumAABB;
    // 由视锥的八个顶点计算视锥包围框box最大最小点（世界坐标系坐标）
    frustum.ComputeBoundingBox(&frustumAABB);
    // 视锥包围框box最大最小点对应的chunk索引
    ChunkID minID = GetIDAt(frustumAABB.min);
    ChunkID maxID = GetIDAt(frustumAABB.max) + Eigen::Vector3i(1, 1, 1);

    //printf("FrustumAABB: %f %f %f %f %f %f\n", frustumAABB.min.x(), frustumAABB.min.y(), frustumAABB.min.z(), frustumAABB.max.x(), frustumAABB.max.y(), frustumAABB.max.z());
    //printf("Frustum min: %d %d %d max: %d %d %d\n", minID.x(), minID.y(), minID.z(), maxID.x(), maxID.y(), maxID.z());
    // 遍历box中的chunk索引
    for (int x = minID(0) - 1; x <= maxID(0) + 1; x++)
    {
        for (int y = minID(1) - 1; y <= maxID(1) + 1; y++)
        {
            for (int z = minID(2) - 1; z <= maxID(2) + 1; z++)
            {
                Vec3 min = Vec3(x * chunkSize(0), y * chunkSize(1), z * chunkSize(2)) * voxelResolutionMeters;
                Vec3 max = min + chunkSize.cast<float>() * voxelResolutionMeters;
                // 创建chunk的包围框
                AABB chunkBox(min, max);
                if (frustum.Intersects(chunkBox))
                {
                    // chunk索引加入list
                    chunkList->push_back(ChunkID(x, y, z));
                }
            }
        }
    }

    //printf("%lu chunks intersect frustum\n", chunkList->size());
}

void ChunkManager::GetChunkIDsIntersecting(const PointCloud &cloud, const Transform &cameraTransform, float truncation, float maxDist, ChunkIDList *chunkList)
{
    assert(!!chunkList);
    chunkList->clear();
    const float roundX = 1.0f / (chunkSize.x() * voxelResolutionMeters);
    const float roundY = 1.0f / (chunkSize.y() * voxelResolutionMeters);
    const float roundZ = 1.0f / (chunkSize.z() * voxelResolutionMeters);
    ChunkMap map;
    Point3 minVal(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max(), -std::numeric_limits<int>::max());
    Point3 maxVal(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
    size_t numPoints = cloud.GetPoints().size();
    size_t i = 0;
    for (const Vec3 &point : cloud.GetPoints())
    {
        Vec3 end = cameraTransform * point;
        Vec3 start = cameraTransform.translation();
        float len = (end - start).norm();

        if (len > maxDist)
        {
            continue;
        }

        Vec3 dir = (end - start).normalized();
        Vec3 truncStart = end - dir * truncation;
        Vec3 truncEnd = end + dir * truncation;
        Vec3 startInt = Vec3(truncStart.x() * roundX, truncStart.y() * roundY, truncStart.z() * roundZ);
        Vec3 endInt = Vec3(truncEnd.x() * roundX, truncEnd.y() * roundY, truncEnd.z() * roundZ);

        Point3List intersectingChunks;
        Raycast(startInt, endInt, minVal, maxVal, &intersectingChunks);

        for (const Point3 &id : intersectingChunks)
        {
            if (map.find(id) == map.end())
                map[id] = ChunkPtr();
        }
    }

    for (const std::pair<ChunkID, ChunkPtr> &it : map)
    {
        chunkList->push_back(it.first);
    }
}

/**
 * @brief 内部
 * @param chunk
 * @param index 体素3D索引
 * @param coords 体素中心世界坐标
 * @param nextMeshIndex 算法线的？
 * @param mesh 网格
 */
void ChunkManager::ExtractInsideVoxelMesh(const ChunkPtr &chunk, const Eigen::Vector3i &index, const Vec3 &coords, VertIndex *nextMeshIndex, Mesh *mesh)
{
    assert(mesh != nullptr);
    Eigen::Matrix<float, 3, 8> cubeCoordOffsets = cubeIndexOffsets.cast<float>() * voxelResolutionMeters;
    Eigen::Matrix<float, 3, 8> cornerCoords;
    Eigen::Matrix<float, 8, 1> cornerSDF;
    bool allNeighborsObserved = true;
    // 遍历，以当前体素为左下角的8个体素
    for (int i = 0; i < 8; ++i)
    {
        Eigen::Vector3i corner_index = index + cubeIndexOffsets.col(i);
        const DistVoxel &thisVoxel = chunk->GetDistVoxel(corner_index.x(), corner_index.y(), corner_index.z());

        // Do not extract a mesh here if one of the corner is unobserved and outside the truncation region.
        if (thisVoxel.GetWeight() <= 1e-15)
        {
            allNeighborsObserved = false;
            break;
        }
        cornerCoords.col(i) = coords + cubeCoordOffsets.col(i);
        cornerSDF(i) = thisVoxel.GetSDF();
    }

    if (allNeighborsObserved)
    {
        MarchingCubes::MeshCube(cornerCoords, cornerSDF, nextMeshIndex, mesh);
        if (MarchingCubes::IsOccupied(cornerSDF))
        {
            Eigen::Vector3f n;
            n.x() = coords.x();
            n.y() = coords.y();
            n.z() = coords.z();
            mesh->grids.push_back(n); // 同于体素坐标
        }
    }
}

/**
 * @brief 边界
 */
void ChunkManager::ExtractBorderVoxelMesh(const ChunkPtr &chunk, const Eigen::Vector3i &index, const Eigen::Vector3f &coordinates, VertIndex *nextMeshIndex, Mesh *mesh)
{
    const Eigen::Matrix<float, 3, 8> cubeCoordOffsets = cubeIndexOffsets.cast<float>() * voxelResolutionMeters;
    Eigen::Matrix<float, 3, 8> cornerCoords;
    Eigen::Matrix<float, 8, 1> cornerSDF;
    bool allNeighborsObserved = true;
    for (int i = 0; i < 8; ++i)
    {
        Eigen::Vector3i cornerIDX = index + cubeIndexOffsets.col(i);
        // 因为是周围的8个，所以要判断一下
        if (chunk->IsCoordValid(cornerIDX.x(), cornerIDX.y(), cornerIDX.z()))
        {
            const DistVoxel &thisVoxel = chunk->GetDistVoxel(cornerIDX.x(), cornerIDX.y(), cornerIDX.z());
            // Do not extract a mesh here if one of the corners is unobserved
            // and outside the truncation region.
            if (thisVoxel.GetWeight() <= 1e-15)
            {
                allNeighborsObserved = false;
                break;
            }
            cornerCoords.col(i) = coordinates + cubeCoordOffsets.col(i);
            cornerSDF(i) = thisVoxel.GetSDF();
        }
        else
        {
            // 确定体素是否存在
            Eigen::Vector3i chunkOffset = Eigen::Vector3i::Zero();// chunk的偏移
            
            // 找到体素在哪个chunk中
            for (int j = 0; j < 3; j++)
            {
                if (cornerIDX(j) < 0)
                {
                    chunkOffset(j) = -1;
                    cornerIDX(j) = chunkSize(j) - 1; // 体素不在当前chunk中，就是相邻chunk的边界体素
                }
                else if (cornerIDX(j) >= chunkSize(j))
                {
                    chunkOffset(j) = 1;
                    cornerIDX(j) = 0; // 体素不在当前chunk中，就是相邻chunk的边界体素
                }
            }

            ChunkID neighborID = chunkOffset + chunk->GetID();

            if (HasChunk(neighborID))
            {
                const ChunkPtr &neighborChunk = GetChunk(neighborID);
                if (!neighborChunk->IsCoordValid(cornerIDX.x(), cornerIDX.y(), cornerIDX.z()))
                {
                    allNeighborsObserved = false;
                    break;
                }

                const DistVoxel &thisVoxel = neighborChunk->GetDistVoxel(cornerIDX.x(), cornerIDX.y(), cornerIDX.z());
                // Do not extract a mesh here if one of the corners is unobserved
                // and outside the truncation region.
                if (thisVoxel.GetWeight() <= 1e-15)
                {
                    allNeighborsObserved = false;
                    break;
                }
                cornerCoords.col(i) = coordinates + cubeCoordOffsets.col(i);
                cornerSDF(i) = thisVoxel.GetSDF();
            }
            else
            {
                allNeighborsObserved = false;
                break;
            }
        }
    }

    if (allNeighborsObserved)
    {
        MarchingCubes::MeshCube(cornerCoords, cornerSDF, nextMeshIndex, mesh);
        if (MarchingCubes::IsOccupied(cornerSDF))
        {
            Eigen::Vector3f n;
            n.x() = coordinates.x();
            n.y() = coordinates.y();
            n.z() = coordinates.z();
            mesh->grids.push_back(n);
        }
    }
}

/**
 * @brief 生成chunk的mesh
 */
void ChunkManager::GenerateMesh(const ChunkPtr &chunk, Mesh *mesh)
{
    assert(mesh != nullptr);

    mesh->Clear();
    const int maxX = chunkSize(0); // 体素个数
    const int maxY = chunkSize(1);
    const int maxZ = chunkSize(2);

    Eigen::Vector3i index;
    VoxelID i = 0;
    VertIndex nextIndex = 0; // 算三角网格法线用的

    // For voxels not bordering the outside, we can use a more efficient function.
    // 遍历chunk中的体素，index是索引
    // 内部
    for (index.z() = 0; index.z() < maxZ - 1; index.z()++)
    {
        for (index.y() = 0; index.y() < maxY - 1; index.y()++)
        {
            for (index.x() = 0; index.x() < maxX - 1; index.x()++)
            {
                i = chunk->GetVoxelID(index.x(), index.y(), index.z());
                ExtractInsideVoxelMesh(chunk, index, centroids.at(i) + chunk->GetOrigin(), &nextIndex, mesh);
            }
        }
    }

    // 边界
    // Max X plane (takes care of max-Y corner as well).
    i = 0;
    index.x() = maxX - 1;
    for (index.z() = 0; index.z() < maxZ - 1; index.z()++)
    {
        for (index.y() = 0; index.y() < maxY; index.y()++)
        {
            i = chunk->GetVoxelID(index.x(), index.y(), index.z());
            ExtractBorderVoxelMesh(chunk, index, centroids.at(i) + chunk->GetOrigin(), &nextIndex, mesh);
        }
    }

    // Max Y plane.
    i = 0;
    index.y() = maxY - 1;
    for (index.z() = 0; index.z() < maxZ - 1; index.z()++)
    {
        for (index.x() = 0; index.x() < maxX - 1; index.x()++)
        {
            i = chunk->GetVoxelID(index.x(), index.y(), index.z());
            ExtractBorderVoxelMesh(chunk, index, centroids.at(i) + chunk->GetOrigin(), &nextIndex, mesh);
        }
    }

    // Max Z plane (also takes care of corners).
    i = 0;
    index.z() = maxZ - 1;
    for (index.y() = 0; index.y() < maxY; index.y()++)
    {
        for (index.x() = 0; index.x() < maxX; index.x()++)
        {
            i = chunk->GetVoxelID(index.x(), index.y(), index.z());
            ExtractBorderVoxelMesh(chunk, index, centroids.at(i) + chunk->GetOrigin(), &nextIndex, mesh);
        }
    }

    //printf("Generated a new mesh with %lu verts, %lu norm, and %lu idx\n", mesh->vertices.size(), mesh->normals.size(), mesh->indices.size());
    assert(mesh->vertices.size() == mesh->normals.size());
    assert(mesh->vertices.size() == mesh->indices.size());
}

/**
 * @param pos mesh顶点（世界坐标）
 * @param dist 体素的sdf值
 * @param grad 体素的sdf值的梯度（由xyz方向的六个体素计算，归一化）
 */
bool ChunkManager::GetSDFAndGradient(const Eigen::Vector3f &pos, double *dist, Eigen::Vector3f *grad)
{
    // 顶点所在体素的坐标
    Eigen::Vector3f posf = Eigen::Vector3f(std::floor(pos.x() / voxelResolutionMeters) * voxelResolutionMeters + voxelResolutionMeters / 2.0f,
                                           std::floor(pos.y() / voxelResolutionMeters) * voxelResolutionMeters + voxelResolutionMeters / 2.0f,
                                           std::floor(pos.z() / voxelResolutionMeters) * voxelResolutionMeters + voxelResolutionMeters / 2.0f);
    if (!GetSDF(posf, dist))
        return false;
    double ddxplus, ddyplus, ddzplus = 0.0;
    double ddxminus, ddyminus, ddzminus = 0.0;
    // xyz方向的六个sdf值
    if (!GetSDF(posf + Eigen::Vector3f(voxelResolutionMeters, 0, 0), &ddxplus))
        return false;
    if (!GetSDF(posf + Eigen::Vector3f(0, voxelResolutionMeters, 0), &ddyplus))
        return false;
    if (!GetSDF(posf + Eigen::Vector3f(0, 0, voxelResolutionMeters), &ddzplus))
        return false;
    if (!GetSDF(posf - Eigen::Vector3f(voxelResolutionMeters, 0, 0), &ddxminus))
        return false;
    if (!GetSDF(posf - Eigen::Vector3f(0, voxelResolutionMeters, 0), &ddyminus))
        return false;
    if (!GetSDF(posf - Eigen::Vector3f(0, 0, voxelResolutionMeters), &ddzminus))
        return false;
    
    *grad = Eigen::Vector3f(ddxplus - ddxminus, ddyplus - ddyminus, ddzplus - ddzminus);
    grad->normalize();
    return true;
}

/**
 * @param posf 体素坐标
 * @param dist 返回sdf值
 */
bool ChunkManager::GetSDF(const Eigen::Vector3f &posf, double *dist)
{
    chisel::ChunkPtr chunk = GetChunkAt(posf);
    if (chunk)
    {
        Eigen::Vector3f relativePos = posf - chunk->GetOrigin();
        Eigen::Vector3i coords = chunk->GetVoxelCoords(relativePos);
        chisel::VoxelID id = chunk->GetVoxelID(coords);
        if (id >= 0 && id < chunk->GetTotalNumVoxels())
        {
            const chisel::DistVoxel &voxel = chunk->GetDistVoxel(id);
            if (voxel.GetWeight() > 1e-12)
            {
                *dist = voxel.GetSDF();
                return true;
            }
        }
        return false;
    }
    else
    {
        return false;
    }
}

/**
 * @brief
 * @param colorPos mesh顶点，世界坐标
 */
Vec3 ChunkManager::InterpolateColor(const Vec3 &colorPos)
{
    const float &x = colorPos(0);
    const float &y = colorPos(1);
    const float &z = colorPos(2);
    // 计算点所在体素3D索引（世界坐标系索引？）
    const int x_0 = static_cast<int>(std::floor(x / voxelResolutionMeters));
    const int y_0 = static_cast<int>(std::floor(y / voxelResolutionMeters));
    const int z_0 = static_cast<int>(std::floor(z / voxelResolutionMeters));
    const int x_1 = x_0 + 1;
    const int y_1 = y_0 + 1;
    const int z_1 = z_0 + 1;

    const ColorVoxel *v_000 = GetColorVoxel(Vec3(x_0, y_0, z_0));
    const ColorVoxel *v_001 = GetColorVoxel(Vec3(x_0, y_0, z_1));
    const ColorVoxel *v_011 = GetColorVoxel(Vec3(x_0, y_1, z_1));
    const ColorVoxel *v_111 = GetColorVoxel(Vec3(x_1, y_1, z_1));
    const ColorVoxel *v_110 = GetColorVoxel(Vec3(x_1, y_1, z_0));
    const ColorVoxel *v_100 = GetColorVoxel(Vec3(x_1, y_0, z_0));
    const ColorVoxel *v_010 = GetColorVoxel(Vec3(x_0, y_1, z_0));
    const ColorVoxel *v_101 = GetColorVoxel(Vec3(x_1, y_0, z_1));
    
    // 如果存在null
    if (!v_000 || !v_001 || !v_011 || !v_111 || !v_110 || !v_100 || !v_010 || !v_101)
    {
        const ChunkID &chunkID = GetIDAt(colorPos);

        if (!HasChunk(chunkID))
        {
            return Vec3(0, 0, 0);
        }
        else
        {
            const ChunkPtr &chunk = GetChunk(chunkID);
            return chunk->GetColorAt(colorPos);
        }
    }

    // 有意义么？（坐标-索引）比上1？
    float xd = (x - x_0) / (x_1 - x_0);
    float yd = (y - y_0) / (y_1 - y_0);
    float zd = (z - z_0) / (z_1 - z_0);
    float red, green, blue = 0.0f;
    {
        float c_00 = v_000->GetRed() * (1 - xd) + v_100->GetRed() * xd;
        float c_10 = v_010->GetRed() * (1 - xd) + v_110->GetRed() * xd;
        float c_01 = v_001->GetRed() * (1 - xd) + v_101->GetRed() * xd;
        float c_11 = v_011->GetRed() * (1 - xd) + v_111->GetRed() * xd;
        float c_0 = c_00 * (1 - yd) + c_10 * yd;
        float c_1 = c_01 * (1 - yd) + c_11 * yd;
        float c = c_0 * (1 - zd) + c_1 * zd;
        red = c / 255.0f;
    }
    {
        float c_00 = v_000->GetGreen() * (1 - xd) + v_100->GetGreen() * xd;
        float c_10 = v_010->GetGreen() * (1 - xd) + v_110->GetGreen() * xd;
        float c_01 = v_001->GetGreen() * (1 - xd) + v_101->GetGreen() * xd;
        float c_11 = v_011->GetGreen() * (1 - xd) + v_111->GetGreen() * xd;
        float c_0 = c_00 * (1 - yd) + c_10 * yd;
        float c_1 = c_01 * (1 - yd) + c_11 * yd;
        float c = c_0 * (1 - zd) + c_1 * zd;
        green = c / 255.0f;
    }
    {
        float c_00 = v_000->GetBlue() * (1 - xd) + v_100->GetBlue() * xd;
        float c_10 = v_010->GetBlue() * (1 - xd) + v_110->GetBlue() * xd;
        float c_01 = v_001->GetBlue() * (1 - xd) + v_101->GetBlue() * xd;
        float c_11 = v_011->GetBlue() * (1 - xd) + v_111->GetBlue() * xd;
        float c_0 = c_00 * (1 - yd) + c_10 * yd;
        float c_1 = c_01 * (1 - yd) + c_11 * yd;
        float c = c_0 * (1 - zd) + c_1 * zd;
        blue = c / 255.0f;
    }

    return Vec3(red, green, blue);
}

const DistVoxel *ChunkManager::GetDistanceVoxel(const Vec3 &pos)
{
    ChunkPtr chunk = GetChunkAt(pos);

    if (chunk.get())
    {
        Vec3 rel = (pos - chunk->GetOrigin());
        return &(chunk->GetDistVoxel(chunk->GetVoxelID(rel)));
    }
    else
        return nullptr;
}

/**
 * @brief 获得体素的颜色
 * @param pos 体素3D索引
 */
const ColorVoxel *ChunkManager::GetColorVoxel(const Vec3 &pos)
{
    ChunkPtr chunk = GetChunkAt(pos);

    if (chunk.get())
    {
        Vec3 rel = (pos - chunk->GetOrigin());// 不应该用pos，应该用坐标才对
        // 修改 Vec3 rel = (pos * voxelResolutionMeters - chunk->GetOrigin());
        const VoxelID &id = chunk->GetVoxelID(rel);
        if (id >= 0 && id < chunk->GetTotalNumVoxels())
        {
            return &(chunk->GetColorVoxel(id));
        }
        else
        {
            return nullptr;
        }
    }
    else
        return nullptr;
}

void ChunkManager::ComputeNormalsFromGradients(Mesh *mesh)
{
    assert(mesh != nullptr);
    double dist;
    Vec3 grad;
    for (size_t i = 0; i < mesh->vertices.size(); i++)
    {
        const Vec3 &vertex = mesh->vertices.at(i);
        if (GetSDFAndGradient(vertex, &dist, &grad))
        {
            float mag = grad.norm();// L2范数
            if (mag > 1e-12)
            {
                mesh->normals[i] = grad * (1.0f / mag);// 更新法线
            }
        }
    }
}

void ChunkManager::ColorizeMesh(Mesh *mesh)
{
    assert(mesh != nullptr);

    mesh->colors.clear();
    mesh->colors.resize(mesh->vertices.size());
    for (size_t i = 0; i < mesh->vertices.size(); i++)
    {
        const Vec3 &vertex = mesh->vertices.at(i);// mesh顶点（世界坐标）
        mesh->colors[i] = InterpolateColor(vertex);
    }
}

void ChunkManager::PrintMemoryStatistics()
{
    float bigFloat = std::numeric_limits<float>::max();

    chisel::AABB totalBounds;
    totalBounds.min = chisel::Vec3(bigFloat, bigFloat, bigFloat);
    totalBounds.max = chisel::Vec3(-bigFloat, -bigFloat, -bigFloat);

    ChunkStatistics stats;
    stats.numKnownInside = 0;
    stats.numKnownOutside = 0;
    stats.numUnknown = 0;
    stats.totalWeight = 0.0f;
    for (const std::pair<ChunkID, ChunkPtr> &chunk : chunks)
    {
        AABB bounds = chunk.second->ComputeBoundingBox();
        for (int i = 0; i < 3; i++)
        {
            totalBounds.min(i) = std::min(totalBounds.min(i), bounds.min(i));
            totalBounds.max(i) = std::max(totalBounds.max(i), bounds.max(i));
        }

        chunk.second->ComputeStatistics(&stats);
    }

    Vec3 ext = totalBounds.GetExtents();
    Vec3 numVoxels = ext * 2 / voxelResolutionMeters;
    float totalNum = numVoxels(0) * numVoxels(1) * numVoxels(2);

    float maxMemory = totalNum * sizeof(DistVoxel) / 1000000.0f;

    size_t currentNum = chunks.size() * (chunkSize(0) * chunkSize(1) * chunkSize(2));
    float currentMemory = currentNum * sizeof(DistVoxel) / 1000000.0f;

    printf("Num Unknown: %lu, Num KnownIn: %lu, Num KnownOut: %lu Weight: %f\n", stats.numUnknown, stats.numKnownInside, stats.numKnownOutside, stats.totalWeight);
    printf("Bounds: %f %f %f %f %f %f\n", totalBounds.min.x(), totalBounds.min.y(), totalBounds.min.z(), totalBounds.max.x(), totalBounds.max.y(), totalBounds.max.z());
    printf("Theoretical max (MB): %f, Current (MB): %f\n", maxMemory, currentMemory);
}

} // namespace chisel
