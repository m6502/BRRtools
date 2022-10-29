project('BRRtools', 'c',
    version : '0.1',
    default_options : ['warning_level=3'],
)

cc = meson.get_compiler('c')

if cc.get_id() == 'gcc'
    WARNINGS = (
        '-Wall -Wextra -Wconversion -Wsign-conversion -Wtype-limits ' +
        '-Wno-unused-function -Wno-unused-parameter ' +
        '-Werror=return-type').split()
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
