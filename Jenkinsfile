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
                 sh 'cd ..'
            	 sh 'mkdir RepoB'
            	 sh 'rsync -av --exclude=".*" RepoA/ RepoB/'
                echo 'Deploy'
            }
        }
    }
}
