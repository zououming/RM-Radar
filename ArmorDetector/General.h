//
// Created by bazinga on 19-10-25.
//

#ifndef RM_NBUT2020_GENERAL_H
#define RM_NBUT2020_GENERAL_H

#pragma once

namespace rm
{

    enum ColorChannels		// 颜色通道
    {
        BLUE = 0,
        GREEN = 1,
        RED = 2
    };

    enum ObjectType				// 目标类型
    {
        UNKNOWN_ARMOR = 0,		// 未检测的装甲板 = 0
        SMALL_ARMOR = 1,		// 小装甲板 = 1
        BIG_ARMOR = 2,			// 大装甲板 = 2
        MINI_RUNE = 3,			// 小符 （用不到）
        GREAT_RUNE = 4			// 大符 （用不到）
    };



}

#endif //RM_NBUT2020_GENERAL_H
