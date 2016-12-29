/*
    This source file is part of Rigs of Rods
    Copyright 2005-2012 Pierre-Michel Ricordel
    Copyright 2007-2012 Thomas Fischer
    Copyright 2013+     Petr Ohlidal & contributors

    For more information, see http://www.rigsofrods.org/

    Rigs of Rods is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3, as
    published by the Free Software Foundation.

    Rigs of Rods is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Rigs of Rods. If not, see <http://www.gnu.org/licenses/>.
*/


/// @file
/// @author Petr Ohlidal
/// @date   05/2015


#pragma once

#include "BitFlags.h"
#include "ForwardDeclarations.h"
#include "Locator_t.h"

#include <OgreVector3.h>
#include <OgreColourValue.h>
#include <vector>

namespace RoR
{

struct FlexBodyRecordHeader
{
    FlexBodyRecordHeader():
        flags(0)
    {}

    int            vertex_count;
    int	           node_center;
    int	           node_x;
    int	           node_y;
    Ogre::Vector3  center_offset;
    int            camera_mode;
    int            shared_buf_num_verts;
    int            num_submesh_vbufs;
    unsigned char  flags;

    BITMASK_PROPERTY(flags, 1, IS_ENABLED,             IsEnabled           , SetIsEnabled            );
    BITMASK_PROPERTY(flags, 2, IS_FAULTY,              IsFaulty            , SetIsFaulty             );
    BITMASK_PROPERTY(flags, 3, USES_SHARED_VERTEX_DATA,UsesSharedVertexData, SetUsesSharedVertexData );
    BITMASK_PROPERTY(flags, 4, HAS_TEXTURE,            HasTexture          , SetHasTexture           );
    BITMASK_PROPERTY(flags, 5, HAS_TEXTURE_BLEND,      HasTextureBlend     , SetHasTextureBlend      );
};

struct FlexBodyCacheData
{
    FlexBodyCacheData():
        dst_pos(nullptr),
        src_normals(nullptr),
        src_colors(nullptr),
        locators(nullptr)
    {}

    // NOTE: No freeing of memory needed, pointers will be copied to FlexBody instances.

    FlexBodyRecordHeader header;

    Ogre::Vector3*    dst_pos;
    Ogre::Vector3*    src_normals;
    Ogre::ARGB*       src_colors;
    Locator_t*        locators; //!< 1 loc per vertex
};

/// Enables saving and loading flexbodies from/to binary file.
///
/// FILE STRUCTURE:
/// 1. Signature
/// 2. Metadata @see FlexBodyFileMetadata
/// 3. Flexbodies
///     a. Header @see FlexBodyRecordHeader
///     b. Data (not present if flexbody has flags IS_FAULTY==true or IS_ENABLED==false)
///         1. Locator list
///         2. Positions buffer
///         3. Normals buffer
///         4. Colors buffer (only present if flag HAS_TEXTURE_BLEND == true)
class FlexBodyFileIO
{
public:
    enum ResultCode
    {
        RESULT_CODE_OK,
        RESULT_CODE_GENERAL_ERROR,
        RESULT_CODE_ERR_FOPEN_FAILED,
        RESULT_CODE_ERR_SIGNATURE_MISMATCH,
        RESULT_CODE_ERR_VERSION_MISMATCH,
        RESULT_CODE_ERR_CACHE_NUMBER_UNDEFINED,
        RESULT_CODE_FREAD_OUTPUT_INCOMPLETE,
        RESULT_CODE_FWRITE_OUTPUT_INCOMPLETE
    };

    static const char*        SIGNATURE;
    static const unsigned int FILE_FORMAT_VERSION = 1;

    FlexBodyFileIO();

    std::vector<FlexBody*> &  GetList();
    inline void               AddItemToSave(FlexBody* fb)     { m_items_to_save.push_back(fb); }
    inline void               SetCacheEntryNumber(int n)      { m_cache_entry_number = n; }
    inline FlexBodyCacheData* GetLoadedItem(unsigned index)   { return & m_loaded_items[index]; }
    ResultCode                SaveFile();
    ResultCode                LoadFile();

private:
    struct FlexBodyFileMetadata
    {
        unsigned int   file_format_version;
        unsigned int   num_flexbodies;
    };

    void        OpenFile(const char* fopen_mode);
    void        WriteToFile(void* source, size_t length);
    void        ReadFromFile(void* dest, size_t length);
    inline void CloseFile()                                 { if (m_file != nullptr) { fclose(m_file); } }
                
    void        WriteSignature();
    void         ReadAndCheckSignature();

    void        WriteMetadata();
    void         ReadMetadata(FlexBodyFileMetadata* meta);

    void        WriteFlexbodyHeader(FlexBody*          flexbody);
    void         ReadFlexbodyHeader(FlexBodyCacheData* flexbody);

    void        WriteFlexbodyLocatorList(FlexBody*          flexbody);
    void         ReadFlexbodyLocatorList(FlexBodyCacheData* flexbody);

    void        WriteFlexbodyNormalsBuffer(FlexBody*          flexbody);
    void         ReadFlexbodyNormalsBuffer(FlexBodyCacheData* flexbody);

    void        WriteFlexbodyPositionsBuffer(FlexBody*          flexbody);
    void         ReadFlexbodyPositionsBuffer(FlexBodyCacheData* flexbody);

    void        WriteFlexbodyColorsBuffer(FlexBody*          flexbody);
    void         ReadFlexbodyColorsBuffer(FlexBodyCacheData* flexbody);

    std::vector<FlexBody*>          m_items_to_save;
    std::vector<FlexBodyCacheData>  m_loaded_items;
    FILE*                           m_file;
    unsigned int                    m_fileformat_version;
    int                             m_cache_entry_number;
};

class FlexFactory
{
public:
    FlexFactory() {}

    FlexFactory(
        MaterialFunctionMapper*   mfm,
        MaterialReplacer*         mat_replacer,
        Skin*                     skin,
        node_t*                   all_nodes,
        bool                      is_flexbody_cache_enabled,
        int                       cache_entry_number = -1
        );

    FlexBody* CreateFlexBody(
        const int num_nodes_in_rig, 
        const char* mesh_name, 
        const char* mesh_unique_name, 
        const int ref_node, 
        const int x_node, 
        const int y_node, 
        Ogre::Vector3 const & offset,
        Ogre::Quaternion const & rot, 
        std::vector<unsigned int> & node_indices
        );

    void           CheckAndLoadFlexbodyCache();
    void           SaveFlexbodiesToCache();

private:
    MaterialFunctionMapper* m_material_function_mapper;
    MaterialReplacer*       m_material_replacer;
    Skin*                   m_used_skin;
    bool                    m_enable_flexbody_LODs;

    FlexBodyFileIO          m_flexbody_cache;
    bool                    m_is_flexbody_cache_enabled;
    bool                    m_is_flexbody_cache_loaded;
    unsigned int            m_flexbody_cache_next_index;

    node_t*                 m_rig_nodes_ptr;
};

} // namespace RoR
