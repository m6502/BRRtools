project('BRRtools', 'c',
    version : '0.1',
    default_options : ['warning_level=3'],
)

cc = meson.get_compiler('c')
id = cc.get_id()

if id == 'gcc' or id == 'clang'
    WARNINGS = (
        '-Wconversion -Wsign-conversion -Wtype-limits ' +
        '-Wno-unused-function -Wno-unused-parameter ' +
        '-Werror=return-type -Werror=int-conversion').split()
    # Omitted: -Wmissing-declarations and Clang-only -Wmissing-variable-declarations
    add_global_arguments(WARNINGS, language : 'c')
endif

m_dep = cc.find_library('m', required : false)

brr_lib = static_library('brr',
    sources : [
        'src/brr.c',
        'src/brr.h',
        'src/common.h',
    ],
)

executable('brr_decoder',
    'src/brr_decoder.c',
    link_with : [brr_lib],
    dependencies : [m_dep],
    install : true,
)

# executable('spc_decoder',
#     'src/spc_decoder.c',
#     link_with : [brr_lib],
#     dependencies : [m_dep],
#     install : true,
# )

executable('brr_encoder',
    'src/brr_encoder.c',
    link_with : [brr_lib],
    dependencies : [m_dep],
    install : true,
)
