package com.outoftheboxrobotics.photoncore

import java.lang.reflect.Field

object ReflectionUtils {
    @JvmStatic
    fun getField(clazz: Class<*>, fieldName: String): Field? {
        try {
            val f = clazz.getDeclaredField(fieldName)
            f.isAccessible = true
            return f
        } catch (e: NoSuchFieldException) {
            val superClass = clazz.superclass
            if (superClass != null) {
                return getField(superClass, fieldName)
            }
        }
        return null
    }

    fun getField(clazz: Class<*>, target: Class<*>): Field? {
        for (f in clazz.declaredFields) {
            if (f.type == target) {
                f.isAccessible = true
                return f
            }
        }
        val superClass = clazz.superclass
        return if (superClass != null) {
            getField(clazz.superclass, target)
        } else {
            null
        }
    }

    fun deepCopy(org: Any, target: Any) {
        val fields = org.javaClass.declaredFields
        for (f in fields) {
            f.isAccessible = true
            val f2 = getField(target.javaClass, f.name)
            if (f2 != null) {
                f2.isAccessible = true
                try {
                    f2[target] = f[org]
                } catch (e: IllegalAccessException) {
                    e.printStackTrace()
                }
            }
        }
    }
}