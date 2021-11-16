import os
import pathlib


needUpdate = {
    r"build\_deps\ulua-src\CMakeLists.txt": {
        "from": ["9fd39fd3075c9ecf2a41f64b1f4440ddaef7586e7f620f1625fd4016dd40ed57"],
        "to": ["e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"]
    },

    r"build\_deps\uantlr-src\CMakeLists.txt": {
        "from": ["906dbec5957da4a39910e97a95b9019deed03bf686b21a36f9781243311fc2d3"],
        "to": ["e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"]
    },

    r"build/_deps/uantlr-src/CMakeLists.txt": {
        "from": ["125868ce7fceed283f7c41687b408067ad19aa33fdfc9eca432456867250ad26"],
        "to": ["e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"]
    },

    r"build/_deps/uluapp-src/include/_deps/CMakeLists.txt": {
        "from": ["8c55aca894fd8e0edde0ed204497b4e41db188969ebd1906f35c5d86cb9f16db"],
        "to": ["e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"]
    },

    r"build/_deps/uecs-src/include/_deps/CMakeLists.txt": {
        "from": ["2b02e44b97bb63568e8c06faf0abe71c32e607ae18da42992c84e000f81bdd9c"],
        "to": ["e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"]
    },

    r"build/_deps/udx12-src/CMakeLists.txt": {
        "from": ["7ad7ff6131b72d223945ba66ac610230dd492d39d58af94e839ced5cc7ca3085"],
        "to": ["e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"]
    },

    r"build/_deps/udx12-src/include/UDX12/_deps/CMakeLists.txt": {
        "from": ["8adafba084b8f2fb5272a81e2b8d8248e14ff5b87531616d6fe55de4a091b327"],
        "to": ["e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"]
    },

    r"build/_deps/utopia-src/CMakeLists.txt": {
        "from": ["1bee5f2424612e3ef3684d139d6038ef2420eac7687bf210057e752efe2a05d5",
                 "2898a66a808a92c546bc5c173423bb5ea84ba0278a897d6f03dcfc1a6f98d866",
                 "e57ba9cd40ca2113bef0bc1864c3b8bc0b9a97c39f50da02aaf251ce0a168f0c"],
        "to": ["e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
               "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
               "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"]
    },

    r"build/_deps/utopia-src/src/Render/CMakeLists.txt": {
        "from": ["a389cbc501da7fb24113ea4d2812c6b5d244ef6bbfc241dffbb9527baa76dc3c"],
        "to": ["e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"]
    },

    r"build/_deps/utopia-src/src/Asset/CMakeLists.txt": {
        "from": ["2b5531d579a1c665a7d7ea1fe396d8f62d6043ccb2a88890ed42fd19b858ae0d"],
        "to": ["e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"]
    },
}


def main():
    root_path_ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # "GAMES102\homeworks\project"
    print("Root Path:", root_path_)
    root_path_ = pathlib.Path(root_path_)
    for k, v in needUpdate.items():
        file_path_ = root_path_ / k
        print("File Path:", file_path_)
        from_list_ = v['from']
        to_list_ = v['to']
        assert len(from_list_) == len(to_list_)
        assert os.path.exists(file_path_)
        with open(file_path_, 'r+') as f:
            d = f.read()
            for i in range(len(from_list_)):
                d = d.replace(from_list_[i].upper(), to_list_[i]).replace(from_list_[i].lower(), to_list_[i])
            f.seek(0, 0)
            f.write(d)


if __name__ == '__main__':
    main()
