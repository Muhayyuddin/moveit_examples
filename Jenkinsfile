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
             sh ' if [ -d "RepositoryB" ]
                 then 
                 echo "Directory /path/to/dir exists." 
                 else 
                 echo "Error: Directory /path/to/dir does not exists." 
                 fi '
                echo 'Deploy'
            }
        }
    }
}
