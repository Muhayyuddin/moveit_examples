pipeline {
    agent any

    stages {
        stage('checkout') {
            steps {
                echo 'Hello World'
            }
        }
         stage('Build') {
            steps {
                echo 'Build'
            }
        }
         stage('Deploy') {
            steps {
                sh ' if [ -d "RepositoryB" ]'
                sh ' then '
                sh '   echo "Directory /path/to/dir exists." '
                sh ' else '
                sh '  echo "Error: Directory /path/to/dir does not exists." '
                sh ' fi '
                echo 'Deploy'
            }
        }
    }
}
