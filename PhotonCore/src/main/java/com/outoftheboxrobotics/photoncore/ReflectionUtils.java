package com.outoftheboxrobotics.photoncore;

import java.lang.reflect.Field;

@SuppressWarnings("rawtypes")
public class ReflectionUtils {
    public static Field getField(Class clazz, String fieldName) {
        try {
            Field field = clazz.getDeclaredField(fieldName);
            field.setAccessible(true);
            return field;
        } catch (NoSuchFieldException e) {
            Class superClass = clazz.getSuperclass();
            if (superClass != null) {
                return getField(superClass, fieldName);
            }
        }
        return null;
    }

    @SuppressWarnings({"unchecked"})
    public static <T> T getFieldValue(Object object, String fieldName)  {
        Field field = getField(object.getClass(), fieldName);
        if (field != null) {
            field.setAccessible(true);
            try{
                return (T)field.get(object);
            }catch (Exception e)
            {
                return null;
            }

        }
        return null;
    }
    public static void setFieldValue(Object object, Object value, String fieldName)  {
        Field field = getField(object.getClass(), fieldName);
        if (field != null) {
            field.setAccessible(true);
            try{
                field.set(object, value);
            }catch (Exception ignored)
            {

            }

        }
    }
    public static void deepCopy(Object org, Object target){
        Field[] fields = org.getClass().getDeclaredFields();
        for(Field f : fields){
            f.setAccessible(true);
            Field f2 = getField(target.getClass(), f.getName());
            if(f2 != null){
                f2.setAccessible(true);
                try {
                    f2.set(target, f.get(org));
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}