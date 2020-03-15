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

#ifndef CHISEL_H_
#define CHISEL_H_

#include <open_chisel/threading/Threading.h>
#include <open_chisel/ChunkManager.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/pointcloud/PointCloud.h>

namespace chisel
{

class Chisel
{
  public:
    Chisel();
    Chisel(const Eigen::Vector3i &chunkSize, float voxelResolution, bool useColor);
    virtual ~Chisel();

    inline const ChunkManager &GetChunkManager() const
    {
        return chunkManager;
    }
    inline ChunkManager &GetMutableChunkManager()
    {
        return chunkManager;
    }
    inline void SetChunkManager(const ChunkManager &manager)
    {
        chunkManager = manager;
    }

    void IntegratePointCloud(const ProjectionIntegrator &integrator, const PointCloud &cloud, const Transform &extrinsic, float truncation, float maxDist);

    template <class DataType>
    void IntegrateDepthScan(const ProjectionIntegrator &integrator, const std::shared_ptr<const DepthImage<DataType>> &depthImage, const Transform &extrinsic, const PinholeCamera &camera)
    {
        printf("CHISEL: Integrating a scan\n");
        Frustum frustum;
        camera.SetupFrustum(extrinsic, &frustum);

        ChunkIDList chunksIntersecting;
        chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);

        std::mutex mutex;
        ChunkIDList garbageChunks;
        for (const ChunkID &chunkID : chunksIntersecting)
        // parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID& chunkID)
        {
            bool chunkNew = false;

            mutex.lock();
            if (!chunkManager.HasChunk(chunkID))
            {
                chunkNew = true;
                chunkManager.CreateChunk(chunkID);
            }

            ChunkPtr chunk = chunkManager.GetChunk(chunkID);
            mutex.unlock();

            bool needsUpdate = integrator.Integrate(depthImage, camera, extrinsic, chunk.get());

            mutex.lock();
            if (needsUpdate)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        for (int dz = -1; dz <= 1; dz++)
                        {
                            meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
                        }
                    }
                }
            }
            else if (chunkNew)
            {
                garbageChunks.push_back(chunkID);
            }
            mutex.unlock();
        }
        //);
        printf("CHISEL: Done with scan\n");
        GarbageCollect(garbageChunks);
        chunkManager.PrintMemoryStatistics();
    }

    /**
     * @brief ......
     * @param integrator 融合器
     * @param depthImage 深度图
     * @param depthExtrinsic 深度图位姿
     * @param depthCamera 深度相机模型
     * @param colorImage 彩色图
     * @param colorExtrinsic 彩色图位姿
     * @param colorCamera 彩色相机模型
     * @return void
    */
    template <class DataType, class ColorType>
    void IntegrateDepthScanColor(const ProjectionIntegrator &integrator, const std::shared_ptr<const DepthImage<DataType>> &depthImage, const Transform &depthExtrinsic, const PinholeCamera &depthCamera, const std::shared_ptr<const ColorImage<ColorType>> &colorImage, const Transform &colorExtrinsic, const PinholeCamera &colorCamera)
    {
        printf("CHISEL: Integrating a color scan\n");
        auto wall_time = std::chrono::system_clock::now();
        Frustum frustum;// 截锥
        depthCamera.SetupFrustum(depthExtrinsic, &frustum);
        
        // 视锥中的chunk3D索引序列，也就是需要更新的
        ChunkIDList chunksIntersecting;
        chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);
        
        std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - wall_time;
        printf("intersecting wall time: %f\n", elapsed.count() * 1000);

        wall_time = std::chrono::system_clock::now();
        int n = chunksIntersecting.size();
        std::vector<bool> isNew(n);
        // ChunkMap = std::unordered_map<ChunkID, ChunkPtr, ChunkHasher>
        // ChunkMap::iterator是map的迭代器
        std::vector<ChunkMap::iterator> newChunks(n); // n个，会不会浪费了好几个空间
        std::vector<bool> isGarbage(n);
        // 如果视锥中还没有此chunk就新建
        for (int i = 0; i < n; i++)
        {
            isNew[i] = false;
            isGarbage[i] = false;
            ChunkID chunkID = chunksIntersecting[i];
            if (!chunkManager.HasChunk(chunkID))
            {
                isNew[i] = true;
                newChunks[i] = chunkManager.CreateChunk(chunkID);
            }
        }
        printf("bucket_count: %d\n", chunkManager.GetBucketSize());
        elapsed = std::chrono::system_clock::now() - wall_time;
        printf("allocation wall time: %f\n", elapsed.count() * 1000);

        wall_time = std::chrono::system_clock::now();

        int nThread = 16;
        std::vector<int> debug_v;
        std::vector<std::thread> threads;
        std::mutex m;
        int blockSize = (n + nThread - 1) / nThread;// 线程分块
        // tsdf融合
        for (int i = 0; i < nThread; i++)
        {
            int s = i * blockSize;// 块内start
            printf("thread: %d, s: %d, blockSize: %d\n", i, s, blockSize);
            threads.push_back(std::thread([s, n, blockSize, this, &m, &chunksIntersecting,
                                           &depthImage, &depthCamera, &depthExtrinsic, &colorImage, &colorCamera, &colorExtrinsic, &integrator,
                                           &isNew, &isGarbage,
                                           &debug_v]()
                                          {
                                              for (int j = 0, k = s; j < blockSize && k < n; j++, k++)
                                              {
                                                  ChunkID chunkID = chunksIntersecting[k];// 3D索引
                                                  ChunkPtr chunk = this->chunkManager.GetChunk(chunkID);
                                                  // 此时没锁会有冲突么？
                                                  // 不会处理的每个chunk内的空间，chunk在线程间是区分的
                                                  bool needsUpdate = integrator.IntegrateColor(depthImage, depthCamera, depthExtrinsic, colorImage
                                                                         , colorCamera, colorExtrinsic, chunk.get());// .get获得指针
                                                  // 没有更新且是新建的chunk
                                                  if (!needsUpdate && isNew[k])
                                                  {
                                                      isGarbage[k] = true;
                                                  }
                                                  // 上锁，因为涉及其他chunk，可能不安全
                                                  if (needsUpdate)
                                                  {
                                                      m.lock();
                                                      // -1,0,1是指要遍历当前chunk周围的chunk？
                                                      for (int dx = -1; dx <= 1; dx++)
                                                      {
                                                          for (int dy = -1; dy <= 1; dy++)
                                                          {
                                                              for (int dz = -1; dz <= 1; dz++)
                                                              {
                                                                  // std::unordered_map<ChunkID, bool, ChunkHasher>
                                                                  this->meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
                                                              }
                                                          }
                                                      }
                                                      m.unlock();
                                                  }
                                              }
                                          }));
        }

        for (int i = 0; i < nThread; i++)
            threads[i].join();

        elapsed = std::chrono::system_clock::now() - wall_time;
        printf("integration wall time: %f\n", elapsed.count() * 1000);

        wall_time = std::chrono::system_clock::now();
        //ChunkIDList garbageChunks;
        for (int i = 0; i < n; i++)
            if (isGarbage[i])
            {
                // 直接erase
                chunkManager.RemoveChunk(newChunks[i]);
                //garbageChunks.push_back(chunksIntersecting[i]);
            }
        //GarbageCollect(garbageChunks);
        printf("CHISEL: Done with color scan\n");
        //chunkManager.PrintMemoryStatistics();
        elapsed = std::chrono::system_clock::now() - wall_time;
        printf("garbage wall time: %f\n", elapsed.count() * 1000);
    }

    void GarbageCollect(const ChunkIDList &chunks);
    void UpdateMeshes();

    bool SaveAllMeshesToPLY(const std::string &filename);
    void Reset();

    const ChunkSet &GetMeshesToUpdate() const
    {
        return meshesToUpdate;
    }

  protected:
    ChunkManager chunkManager;
    ChunkSet meshesToUpdate; // 需要更新的chunk，在一个chunk中融合数据后，也会更新周围的8个chunk
};
typedef std::shared_ptr<Chisel> ChiselPtr;
typedef std::shared_ptr<const Chisel> ChiselConstPtr;

} // namespace chisel

#endif // CHISEL_H_
