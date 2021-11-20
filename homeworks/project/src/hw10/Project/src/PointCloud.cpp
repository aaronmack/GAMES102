#include "PointCloud.h"
PointCloud::PointCloud(){}
bool PointCloud::load(const string& path) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path,aiProcess_JoinIdenticalVertices);
    if (scene) {
        aiMesh* m = scene->mMeshes[0];
        unsigned int vcnt = m->mNumVertices;
       // cout << "vcnt:"<<vcnt << endl;     
        posList.resize(vcnt);
        for (int i = 0; i < vcnt; i++) {
            float x = m->mVertices[i].x;
            float y = m->mVertices[i].y;
            float z = m->mVertices[i].z;
            posList[i] = vec3(x, y, z);
        }
        glGenVertexArrays(1,&vao);
        glBindVertexArray(vao);
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, posList.size() * 12, posList.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(0);
        return true;
    }
    else {
        cout << "model not found" << endl;
        return false;
    }
}
void PointCloud::draw() {
    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, 0, posList.size());
}
void PointCloud::draw(int off,int cnt) {
    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, off,cnt);
}