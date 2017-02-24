{
    "targets": [
        {
            "target_name": "path-finder",
            "sources": [
                "src/path_finder.cpp",
                "src/path_finder_wrapper.cpp",
                "src/wrapper_util.cpp"
            ],
            "include_dirs": [
                "<!(node -e \"require('nan')\")"
            ],
            'conditions': [
                ['OS!="win"', {
                    'cflags_cc': [
                        '-fexceptions'
                    ]
                }],
                ['OS=="mac"', {
                    'xcode_settings': {
                        'CLANG_CXX_LANGUAGE_STANDARD': 'c++11',
                        'CLANG_CXX_LIBRARY': 'libc++',
                        'GCC_ENABLE_CPP_EXCEPTIONS': 'YES',
                        'MACOSX_DEPLOYMENT_TARGET': '10.7'
                    }
                }]
            ]
        }
    ]
}
