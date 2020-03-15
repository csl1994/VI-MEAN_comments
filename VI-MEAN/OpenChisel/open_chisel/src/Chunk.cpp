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

#include <open_chisel/Chunk.h>

namespace chisel
{

    Chunk::Chunk()
    {
        // TODO Auto-generated constructor stub

    }
 
    /**
     * @brief 构造函数，chunkmanager中CreateChunk调用
     */
    Chunk::Chunk(const ChunkID id, const Eigen::Vector3i& nv, float r, bool useColor) :
            ID(id), numVoxels(nv), voxelResolutionMeters(r)
    {
        AllocateDistVoxels();

        if(useColor)
        {
            AllocateColorVoxels();
        }
        // 原点，由索引生成世界坐标系3D坐标
        origin = Vec3(numVoxels(0) * ID(0) * voxelResolutionMeters, numVoxels(1) * ID(1) * voxelResolutionMeters, numVoxels(2) * ID(2) * voxelResolutionMeters);
    }

    Chunk::~Chunk()
    {}
    
    /**
     * @brief 分配体素空间
     */
    void Chunk::AllocateDistVoxels()
    {
        int totalNum = GetTotalNumVoxels();
        voxels.clear();
        // void resize (size_type n, value_type val = value_type());
        // n: New container size, expressed in number of elements. Member type size_type is an unsigned integral type.
        // val:Object whose content is copied to the added elements in case that n is greater than the current container size.
        // If not specified, the default constructor is used instead. Member type value_type is the type of the elements in the container, 
        // defined in vector as an alias of the first template parameter (T).
        voxels.resize(totalNum, DistVoxel());
    }
    
    /**
     * @brief 分配色彩空间
     */
    void Chunk::AllocateColorVoxels()
    {
        int totalNum = GetTotalNumVoxels();
        colors.clear();
        colors.resize(totalNum, ColorVoxel());
    }

    AABB Chunk::ComputeBoundingBox()
    {
        Vec3 pos = origin;
        Vec3 size = numVoxels.cast<float>() * voxelResolutionMeters;
        return AABB(pos, pos + size);
    }
    
    /**
     * @brief 计算体素在chunk的位置
     */
    Point3 Chunk::GetVoxelCoords(const Vec3& worldCoords) const
    {
        const float roundingFactorX = 1.0f / (voxelResolutionMeters);
        const float roundingFactorY = 1.0f / (voxelResolutionMeters);
        const float roundingFactorZ = 1.0f / (voxelResolutionMeters);
        return Point3( static_cast<int>(std::floor(worldCoords(0) * roundingFactorX)),
                       static_cast<int>(std::floor(worldCoords(1) * roundingFactorY)),
                       static_cast<int>(std::floor(worldCoords(2) * roundingFactorZ)));
    }
    
    /**
     * @brief 获得点坐在体素的1D索引
     * @param worldPos还是chunk内坐标？
     */
    VoxelID Chunk::GetVoxelID(const Vec3& worldPos) const
    {
        std::cout << "[DEBUG] GetVoxelID param worldPos (3Dindex): " << worldPos << std::endl;
        return GetVoxelID(GetVoxelCoords(worldPos));
    }


    void Chunk::ComputeStatistics(ChunkStatistics* stats)
    {
        assert(stats != nullptr);

        for (const DistVoxel& vox : voxels)
        {
            float weight = vox.GetWeight();
            if (weight > 0)
            {
                float sdf = vox.GetSDF();
                if (sdf < 0)
                {
                    stats->numKnownInside++;
                }
                else
                {
                    stats->numKnownOutside++;
                }
            }
            else
            {
                stats->numUnknown++;
            }

            stats->totalWeight += weight;

        }
    }

    /**
     * @brief 
     * @param pos 世界坐标系点
     */
    Vec3 Chunk::GetColorAt(const Vec3& pos)
    {
        if(ComputeBoundingBox().Contains(pos))
        {
            Vec3 chunkPos = (pos - origin) / voxelResolutionMeters;
            int chunkX = static_cast<int>(chunkPos(0));
            int chunkY = static_cast<int>(chunkPos(1));
            int chunkZ = static_cast<int>(chunkPos(2));

            if(IsCoordValid(chunkX, chunkY, chunkZ))
            {
                const ColorVoxel& color = GetColorVoxel(chunkX, chunkY, chunkZ);
                float maxVal = static_cast<float>(std::numeric_limits<uint8_t>::max());
                // maxVal是为了什么？
                return Vec3(static_cast<float>(color.GetRed()) / maxVal, static_cast<float>(color.GetGreen()) / maxVal, static_cast<float>(color.GetBlue()) / maxVal);
            }
        }

        return Vec3::Zero();
    }


} // namespace chisel 
