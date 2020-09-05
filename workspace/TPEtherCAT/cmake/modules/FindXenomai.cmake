 set( XenomaiFound FALSE )
 set( XENOMAI_SEARCH_PATH /usr/local/xenomai /usr/xenomai /usr/xenomai/include)
 find_path(
   XENOMAI_DIR
   include/xeno_config.h
   ${XENOMAI_SEARCH_PATH})

 
 MESSAGE(STATUS "Xenomai found: \"${XENOMAI_DIR}\"")

 #si le fichier xeno-config.h a été trouvé ?
 if(XENOMAI_DIR)
   
   #set the include directory
   set( XENOMAI_INCLUDE_DIR ${XENOMAI_DIR}/include)
   
   execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=native --cflags
     OUTPUT_VARIABLE XENO_CFLAGS
     OUTPUT_STRIP_TRAILING_WHITESPACE)
   execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=native --ldflags
     OUTPUT_VARIABLE XENO_LDFLAGS
     OUTPUT_STRIP_TRAILING_WHITESPACE)
   set(XENOMAI_INCLUDE_DIR ${XENOMAI_DIR}/include)
   set( XenomaiFound TRUE )
 else (XENOMAI_DIR)
   MESSAGE(STATUS "Xenomai NOT Found. (${XENOMAI_SEARCH_PATH})")    
 endif(XENOMAI_DIR)
